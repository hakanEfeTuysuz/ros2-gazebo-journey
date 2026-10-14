import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_robot_description'
    
    # Xacro dosyasının yolunu bul ve parse et
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'my_robot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    
    # Robot State Publisher: URDF'i ROS 2 ağına topic olarak yayınlar
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}]
    )
    
    # Boş bir Gazebo dünyası başlat
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    
    # Robotumuzu Gazebo dünyasında doğur
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'my_robot'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity
    ])