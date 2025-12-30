#!/usr/bin/env python3
"""
Modified dddmr_navigation.launch.py with map loading support
启用MCL定位并配置地图加载
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    mcl_3dl_pkg = get_package_share_directory('mcl_3dl')
    p2p_move_base_pkg = get_package_share_directory('p2p_move_base')
    
    # Arguments for configuration files
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(p2p_move_base_pkg, 'config', 'dddmr_navigation.yaml'),
        description='Path to the navigation config file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(p2p_move_base_pkg, 'rviz', 'p2p_move_base_localization.rviz'),
        description='Path to the RViz config file'
    )
    
    # NEW: Argument for map file location
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(os.path.expanduser('~'), 
                                   'lego_ddd_nav_ws', 'src', 'dddmr_navigation_ws', 'src', 'lego_map', 'map.pcd'),
        description='Path to the PCD map file for localization (from lego_map folder)'
    )

    # Include Localization Launch (NOW ENABLED)
    # localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(mcl_3dl_pkg, 'launch', 'mcl_3dl.launch.py')
    #     ),
    #     launch_arguments=[
    #         ('pcd_file', LaunchConfiguration('map_file')),
    #     ]
    # )

    # Global Planner Node
    global_planner_node = ExecuteProcess(
        cmd=['ros2', 'run', 'global_planner', 'global_planner_node', 
             '--ros-args', '--params-file', LaunchConfiguration('config_file')],
        output='screen'
    )

    # P2P Move Base Node (includes Local Planner, Controller, etc.)
    p2p_move_base_node = ExecuteProcess(
        cmd=['ros2', 'run', 'p2p_move_base', 'p2p_move_base_node', 
             '--ros-args', '--params-file', LaunchConfiguration('config_file'),
             '-p', 'use_twist_stamped:=false'],
        output='screen'
    )
    
    # Clicked Point to Goal Converter (Optional helper)
    clicked2goal_node = Node(
        package='p2p_move_base',
        executable='clicked2goal.py',
        name='clicked2goal',
        output='screen'
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )

    return LaunchDescription([
        config_file_arg,
        rviz_config_arg,
        map_file_arg,               # NEW: Map file argument
        #localization_launch,        # NOW ENABLED: MCL localization
        global_planner_node,
        p2p_move_base_node,
        clicked2goal_node,
        rviz_node
    ])
