import os
import xacro

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Henter path for URDF filer
    pkg_dir = get_package_share_directory('qube_description')
    xacro_path = os.path.join(pkg_dir, 'urdf', 'qube.urdf.xacro')

    # Konverterer xacro til xml -- Command() skapte problemer
    doc = xacro.process_file(xacro_path)
    urdf_xml = doc.toxml()

    return LaunchDescription([
        # Robot state publisher node -- publiserer TF av robot modell til /robot_description fra qube URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_xml}],
            output='screen',
        ),

        # Joint state publisher med GUI -- publiserer input meldinger til /joint_states som RSP leser
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        
        # Rviz2 for visualisering -- config fra view_qube.rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'view_qube.rviz')],
            parameters=[{'use_sim_time': False}],
        )
    ])

