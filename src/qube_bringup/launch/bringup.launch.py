import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Henter paths
    bringup_pkg = get_package_share_directory('qube_bringup')
    driver_pkg = get_package_share_directory('qube_driver')
    desc_pkg = get_package_share_directory('qube_description')
    urdf_path   = os.path.join(bringup_pkg, 'urdf', 'controlled_qube.urdf.xacro')
    
    # Setter opp driver argumenter -- Gjør det mulig å endre de etter oppstart
    baud_arg   = DeclareLaunchArgument('baud_rate',   default_value='115200')
    dev_arg    = DeclareLaunchArgument('device',      default_value='/dev/ttyACM0')
    sim_arg    = DeclareLaunchArgument('simulation',  default_value='true')
    
    # Legger til launch av Qube driver
    launch_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(driver_pkg, 'launch', 'qube_driver.launch.py')
        )
    )

    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_path, ' ',
            'device:=', LaunchConfiguration('device'), ' ',
            'baud_rate:=', LaunchConfiguration('baud_rate'), ' ',
            'simulation:=', LaunchConfiguration('simulation')
        ]),
        value_type=str
    )
    
    # Robot state publisher node -- publiserer TF av robot modell fra controlled_qube URDF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description
          }],
        output='screen'
    )
    
    # Rviz2 for visualisering -- config fra view_qube.rviz i qube_description
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(desc_pkg, 'rviz', 'view_qube.rviz')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        baud_arg, dev_arg, sim_arg,
        launch_driver,
        rsp,
        rviz,
    ])

