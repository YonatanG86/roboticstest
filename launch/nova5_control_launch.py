from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_nova5_control',
            executable='nova5_control_node',
            name='nova5_control_node'
        ),
        Node(
            package='ros_nova5_control',
            executable='nova5_status_node',
            name='nova5_status_node'
        )
    ])
