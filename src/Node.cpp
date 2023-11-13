/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/t07_robot/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <t07_robot/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace t07
{

/**************************************************************************************
 * CTOR/DTOR 
 **************************************************************************************/

Node::Node()
: rclcpp::Node("t07_robot_node")
, _node_heap{}
, _node_hdl{_node_heap.data(),
            _node_heap.size(),
            [this] () { return micros(); },
            [this] (CanardFrame const & frame) { return _can_mgr->transmit(frame); },
            cyphal::Node::DEFAULT_NODE_ID,
            CYPHAL_TX_QUEUE_SIZE,
            CYPHAL_RX_QUEUE_SIZE,
            cyphal::Node::DEFAULT_MTU_SIZE}
, _node_mtx{}
, _node_start{std::chrono::steady_clock::now()}
, _motor_left_qos_profile
{
  rclcpp::KeepLast(1),
  rmw_qos_profile_sensor_data
}
, _motor_left_target{0. * m/s}
, _motor_right_qos_profile
{
  rclcpp::KeepLast(1),
  rmw_qos_profile_sensor_data
}
, _motor_right_target{0. * m/s}
{
  init_cyphal_heartbeat();
  init_cyphal_node_info();

  declare_parameter("can_iface", "can0");
  declare_parameter("can_node_id", 100);

  RCLCPP_INFO(get_logger(),
              "configuring CAN bus:\n\tDevice: %s\n\tNode Id: %ld",
              get_parameter("can_iface").as_string().c_str(),
              get_parameter("can_node_id").as_int());

  _node_hdl.setNodeId(get_parameter("can_node_id").as_int());

  _can_mgr = std::make_unique<CanManager>(
    get_logger(),
    get_parameter("can_iface").as_string(),
    [this](CanardFrame const & frame)
    {
      std::lock_guard<std::mutex> lock(_node_mtx);
      _node_hdl.onCanFrameReceived(frame);
    });

  init_motor_left();
  init_motor_right();

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

Node::~Node()
{
  RCLCPP_INFO(get_logger(), "%s shut down successfully.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_cyphal_heartbeat()
{
  _cyphal_heartbeat_pub = _node_hdl.create_publisher<uavcan::node::Heartbeat_1_0>(1*1000*1000UL /* = 1 sec in usecs. */);
}

void Node::init_cyphal_node_info()
{
  _cyphal_node_info = _node_hdl.create_node_info(
    /* uavcan.node.Version.1.0 protocol_version */
    1, 0,
    /* uavcan.node.Version.1.0 hardware_version */
    1, 0,
    /* uavcan.node.Version.1.0 software_version */
    0, 1,
    /* saturated uint64 software_vcs_revision_id */
#ifdef CYPHAL_NODE_INFO_GIT_VERSION
    CYPHAL_NODE_INFO_GIT_VERSION,
#else
    0,
#endif
    /* saturated uint8[16] unique_id */
    std::array<uint8_t, 16>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F},
    /* saturated uint8[<=50] name */
    "107-systems.t07"
  );
}

CanardMicrosecond Node::micros()
{
  auto const now = std::chrono::steady_clock::now();
  auto const node_uptime = (now - _node_start);
  return std::chrono::duration_cast<std::chrono::microseconds>(node_uptime).count();
}

void Node::init_motor_left()
{
  declare_parameter("motor_left_topic", "/motor/left/target");
  declare_parameter("motor_left_topic_deadline_ms", 100);
  declare_parameter("motor_left_topic_liveliness_lease_duration", 1000);
  declare_parameter("motor_left_pwm_port_id", 600);

  auto const motor_left_topic = get_parameter("motor_left_topic").as_string();
  auto const motor_left_topic_deadline = std::chrono::milliseconds(get_parameter("motor_left_topic_deadline_ms").as_int());
  auto const motor_left_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("motor_left_topic_liveliness_lease_duration").as_int());

  _motor_left_qos_profile.deadline(motor_left_topic_deadline);
  _motor_left_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _motor_left_qos_profile.liveliness_lease_duration(motor_left_topic_liveliness_lease_duration);

  _motor_left_sub_options.event_callbacks.deadline_callback =
    [this, motor_left_topic](rclcpp::QOSDeadlineRequestedInfo & /* event */) -> void
    {
      _motor_left_target = 0. * m/s;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(),5*1000UL,
                            "deadline missed for \"%s\", limiting _motor_left_target to %f m/s.",
                            motor_left_topic.c_str(), _motor_left_target.numerical_value_in(m/s));
    };

  _motor_left_sub_options.event_callbacks.liveliness_callback =
    [this, motor_left_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count == 0)
      {
        _motor_left_target = 0. * m/s;
        RCLCPP_ERROR(get_logger(),
                     "liveliness lost for \"%s\", limiting _motor_left_target to %f m/s",
                     motor_left_topic.c_str(), _motor_left_target.numerical_value_in(m/s));
      }
    };

  _motor_left_sub = create_subscription<std_msgs::msg::Float32>(
    motor_left_topic,
    _motor_left_qos_profile,
    [this](std_msgs::msg::Float32::SharedPtr const msg) { _motor_left_target = static_cast<double>(msg->data) * m/s; },
    _motor_left_sub_options
  );

  _motor_left_pwm_pub = _node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(get_parameter("motor_left_pwm_port_id").as_int(), 1*1000*1000UL);

  _motor_left_ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->motor_left_ctrl_loop(); });
}

void Node::motor_left_ctrl_loop()
{
  if (_motor_left_target > 1. * m/s)
    _motor_left_target = 1. * m/s;

  int16_t const motor_left_pwm_value = _motor_left_target.numerical_value_in(m/s) * 100;

  uavcan::primitive::scalar::Integer16_1_0 pwm_left_msg;
  pwm_left_msg.value = motor_left_pwm_value;

  {
    std::lock_guard <std::mutex> lock(_node_mtx);
    _motor_left_pwm_pub->publish(pwm_left_msg);
    _node_hdl.spinSome();
  }
}

void Node::init_motor_right()
{
  declare_parameter("motor_right_topic", "/motor/right/target");
  declare_parameter("motor_right_topic_deadline_ms", 100);
  declare_parameter("motor_right_topic_liveliness_lease_duration", 1000);
  declare_parameter("motor_right_pwm_port_id", 600);

  auto const motor_right_topic = get_parameter("motor_right_topic").as_string();
  auto const motor_right_topic_deadline = std::chrono::milliseconds(get_parameter("motor_right_topic_deadline_ms").as_int());
  auto const motor_right_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("motor_right_topic_liveliness_lease_duration").as_int());

  _motor_right_qos_profile.deadline(motor_right_topic_deadline);
  _motor_right_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _motor_right_qos_profile.liveliness_lease_duration(motor_right_topic_liveliness_lease_duration);

  _motor_right_sub_options.event_callbacks.deadline_callback =
    [this, motor_right_topic](rclcpp::QOSDeadlineRequestedInfo & /* event */) -> void
    {
      _motor_right_target = 0. * m/s;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(),5*1000UL,
                            "deadline missed for \"%s\", limiting _motor_right_target to %f m/s.",
                            motor_right_topic.c_str(), _motor_right_target.numerical_value_in(m/s));
    };

  _motor_right_sub_options.event_callbacks.liveliness_callback =
    [this, motor_right_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count == 0)
      {
        _motor_right_target = 0. * m/s;
        RCLCPP_ERROR(get_logger(),
                     "liveliness lost for \"%s\", limiting _motor_right_target to %f m/s",
                     motor_right_topic.c_str(), _motor_right_target.numerical_value_in(m/s));
      }
    };

  _motor_right_sub = create_subscription<std_msgs::msg::Float32>(
    motor_right_topic,
    _motor_right_qos_profile,
    [this](std_msgs::msg::Float32::SharedPtr const msg) { _motor_right_target = static_cast<double>(msg->data) * m/s; },
    _motor_right_sub_options
  );

  _motor_right_pwm_pub = _node_hdl.create_publisher<uavcan::primitive::scalar::Integer16_1_0>(get_parameter("motor_right_pwm_port_id").as_int(), 1*1000*1000UL);

  _motor_right_ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->motor_right_ctrl_loop(); });
}

void Node::motor_right_ctrl_loop()
{
  if (_motor_right_target > 1. * m/s)
    _motor_right_target = 1. * m/s;

  int16_t const motor_right_pwm_value = _motor_right_target.numerical_value_in(m/s) * 100;

  uavcan::primitive::scalar::Integer16_1_0 pwm_right_msg;
  pwm_right_msg.value = motor_right_pwm_value;

  {
    std::lock_guard <std::mutex> lock(_node_mtx);
    _motor_right_pwm_pub->publish(pwm_right_msg);
    _node_hdl.spinSome();
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
