"""
Tek komutla: Gazebo + Nav2 + (isteğe bağlı) RViz.

Kullanım:
ros2 launch my_robot_description bringup.launch.py
ros2 launch my_robot_description bringup.launch.py gui:=false rviz:=false  (headless)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    nav2_share = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(pkg_share, 'launch')

    declare_args = [
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]

    # Her include kendi kapsamında: gazebo_ros'un gzserver.launch.py'si de
    # 'params_file' argümanı tanımlıyor (varsayılanı ''). Kapsam olmadan bu
    # değer Nav2'nin params_file'ını ezer.
    sim = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'gazebo.launch.py')),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items(),
    )])

    navigation = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation.launch.py')),
    )])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(declare_args + [sim, navigation, rviz])
