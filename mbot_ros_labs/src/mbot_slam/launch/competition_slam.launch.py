
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mbot_slam',
            executable='slam_node',
            name='mbot_slam_node',
            output='screen'
        ),
    ])
