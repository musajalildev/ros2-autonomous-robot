from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='com2009_team09_2026', 
            executable='obstacle_detection.py', 
            name='obstacle_detection' 
        ),
        Node(
            package='com2009_team09_2026', 
            executable='explore_map.py', 
            name='map_explorer'
             
        )
    ])