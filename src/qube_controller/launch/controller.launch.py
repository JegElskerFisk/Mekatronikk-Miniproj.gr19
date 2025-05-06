import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    sim_arg = DeclareLaunchArgument('simulation', default_value='false')
    
    # PID node -- leser av joint states og publiserer til velocity_controller   
    PID_node = Node(
            package='qube_controller',
            executable='PID_controller',
            name='pid_controller',
            parameters=[{
                'kp': 1.0,
                'ki': 0.0,
                'kd': 0.1,
                'setpoint': 3.0,
                'use_sim_time': False
            }],
            output='screen'
        )
        
    return LaunchDescription([
        sim_arg,
        PID_node,
    ])

