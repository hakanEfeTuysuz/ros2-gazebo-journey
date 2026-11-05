"""Gazebo Classic + robot_state_publisher + robotu dünyaya ekleme."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false: sadece gzserver (headless)'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg_share, 'worlds', 'test_world.world')),
        # Harita (0, 0) noktasında çıkarıldı; AMCL başlangıç pozu da buna göre.
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
    ]

    robot_description = ParameterValue(
        Command(['xacro ', os.path.join(pkg_share, 'urdf', 'my_robot.urdf.xacro')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world, 'gui': gui}.items(),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description', '-entity', 'my_robot',
            '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    return LaunchDescription(declare_args + [gazebo, robot_state_publisher, spawn_entity])
