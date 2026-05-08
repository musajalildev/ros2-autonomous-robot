from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            name='target_beacon',
            default_value='blue',
            description='Colour of the beacon to search for (yellow/red/green/blue)'
        ),

        # SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('tuos_tb3_tools'),
                    'launch',
                    'slam.launch.py'
                ])
            ),
            launch_arguments={'environment': 'real'}.items()
        ),

        # Map saver server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('nav2_map_server'),
                    'launch',
                    'map_saver_server.launch.py'
                ])
            )
        ),

        # Explorer (drives robot + obstacle avoidance + zone tracking)
        Node(
            package='com2009_team09_2026',
            executable='task3_explorer.py',
            name='task3_explorer',
            output='screen'
        ),

        Node(
            package='com2009_team09_2026',
            executable='task3_obstacle_detection.py',
            name='task3_obstacle_detection',
            output='screen'
        ),

        # Beacon search (prints target colour log message)
        Node(
            package='com2009_team09_2026',
            executable='task3_beacon.py',
            name='task3_beacon',
            parameters=[{
                'target_colour': LaunchConfiguration('target_beacon')
            }]
        ),

        # Map saver node
        Node(
            package='com2009_team09_2026',
            executable='task3_map_saver.py',
            name='task3_map_saver',
            output='screen',
        ),

    ])