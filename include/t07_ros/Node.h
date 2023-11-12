/**
 * Copyright (c) 2023 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/t07_ros/graphs/contributors.
 */

#pragma once

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <cyphal++/cyphal++.h>

#include <mp-units/systems/si/si.h>

#include "CanManager.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::s;

namespace t07
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
   Node();
  ~Node();

private:
  static size_t constexpr CYPHAL_O1HEAP_SIZE = (cyphal::Node::DEFAULT_O1HEAP_SIZE * 16);
  static size_t constexpr CYPHAL_TX_QUEUE_SIZE = 256;
  static size_t constexpr CYPHAL_RX_QUEUE_SIZE = 256;
  cyphal::Node::Heap<CYPHAL_O1HEAP_SIZE> _node_heap;
  cyphal::Node _node_hdl;
  std::mutex _node_mtx;
  std::chrono::steady_clock::time_point const _node_start;
  std::unique_ptr<CanManager> _can_mgr;

  cyphal::Publisher<uavcan::node::Heartbeat_1_0> _cyphal_heartbeat_pub;
  std::chrono::steady_clock::time_point _prev_heartbeat_timepoint;
  static std::chrono::milliseconds constexpr CYPHAL_HEARTBEAT_PERIOD{1000};
  void init_cyphal_heartbeat();

  cyphal::NodeInfo _cyphal_node_info;
  void init_cyphal_node_info();

  CanardMicrosecond micros();

  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};

  rclcpp::QoS _motor_left_qos_profile;
  rclcpp::SubscriptionOptions _motor_left_sub_options;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _motor_left_sub;
  quantity<m/s> _motor_left_target;
  cyphal::Publisher<uavcan::primitive::scalar::Integer16_1_0> _motor_left_pwm_pub;
  rclcpp::TimerBase::SharedPtr _motor_left_ctrl_loop_timer;
  void init_motor_left();
  void motor_left_ctrl_loop();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* t07 */
