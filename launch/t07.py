from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
    Node(
      package='t07_ros',
      executable='t07_ros_node',
      name='t07_ros',
      namespace='t07',
      output='screen',
      emulate_tty=True,
      parameters=[
        {'can_iface' : 'can0'},
        {'can_node_id' : 100},
      ]
    )
  ])
