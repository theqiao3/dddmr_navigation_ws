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
    
    # Arguments
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

    # Include Localization Launch
    # localization_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(mcl_3dl_pkg, 'launch', 'mcl_3dl.launch.py')
    #     )
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
             '--ros-args', '--params-file', LaunchConfiguration('config_file')],
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
        #localization_launch,
        global_planner_node,
        p2p_move_base_node,
        clicked2goal_node,
        rviz_node
    ])
