"""Nav2 (AMCL + map_server + planner/controller) bu paketin harita ve parametreleriyle.

Davranış ağacı yolu burada paket dizininden çözülür; nav2_params.yaml
içinde mutlak yol tutulmaz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pkg_share, 'maps', 'my_world_map.yaml')),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('autostart', default_value='true'),
    ]

    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(
                pkg_share, 'behavior_trees', 'navigate_to_pose_backup_first.xml'),
        },
        convert_types=True,
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': use_sim_time,
            'params_file': configured_params,
            'autostart': LaunchConfiguration('autostart'),
        }.items(),
    )

    return LaunchDescription(declare_args + [bringup])
