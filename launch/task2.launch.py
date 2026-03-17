from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='com2009_team09_2026',
            executable='task2.py',
            name='task2_explorer',
            output='screen',
        )
    ])