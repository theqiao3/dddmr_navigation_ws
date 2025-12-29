import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mcl_3dl')
    
    # Arguments
    pcd_file_arg = DeclareLaunchArgument(
        'pcd_file',
        default_value='/home/tianbot/slash_ws/src/slash_navigation/slash_nav2/PCD/test1.pcd',
        description='Path to the PCD map file'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'mcl_3dl_ros2.yaml'),
        description='Path to the config file'
    )

    # Nodes
    pcl_publisher_node = Node(
        package='mcl_3dl',
        executable='pcl_publisher',
        name='pcl_publisher',
        output='screen',
        parameters=[{
            'map_dir': LaunchConfiguration('pcd_file'),
            'ground_dir': LaunchConfiguration('pcd_file'), # Use same map for ground for now
            'global_frame': 'map',
            'map_down_sample': 0.2,
            'ground_down_sample': 0.2
        }]
    )

    mcl_3dl_node = ExecuteProcess(
        cmd=['ros2', 'run', 'mcl_3dl', 'mcl_3dl',
             '--ros-args', 
             '--params-file', LaunchConfiguration('config_file'),
             '-r', 'odom:=/odometry/filtered',
             '-r', 'cloud:=/livox/lidar/pointcloud'
            ],
        output='screen'
    )

    return LaunchDescription([
        pcd_file_arg,
        config_file_arg,
        pcl_publisher_node,
        mcl_3dl_node
    ])
