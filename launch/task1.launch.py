from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    return LaunchDescription([ 
        Node( 
            package='com2009_team09_2026', 
            executable='task1.py', 
            name='velocity_control' 
        )
    ])