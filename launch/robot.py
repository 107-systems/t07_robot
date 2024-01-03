from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='t07_robot',
      executable='t07_robot_node',
      name='t07_robot',
      namespace='t07',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'can_iface' : 'can0'},
        {'can_node_id' : 100},
        {'motor_left_topic': '/motor/left/target'},
        {'motor_left_topic_deadline_ms': 200},
        {'motor_left_topic_liveliness_lease_duration': 1000},
        {'motor_left_rpm_port_id': 1000},
        {'motor_right_topic': '/motor/right/target'},
        {'motor_right_topic_deadline_ms': 200},
        {'motor_right_topic_liveliness_lease_duration': 1000},
        {'motor_right_rpm_port_id': 1001},
      ]
    )
  ])
