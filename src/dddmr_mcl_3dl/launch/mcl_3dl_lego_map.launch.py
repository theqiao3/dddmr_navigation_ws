#!/usr/bin/env python3
"""
MCL 3DL独立定位启动文件 - 针对lego_map文件夹的地图

功能:
  - 单独启动MCL_3DL定位模块
  - 使用lego_map文件夹中的地图进行定位
  - 支持动态参数配置
  - 适合调试和单独测试定位功能

用法:
  # 使用默认配置启动
  ros2 launch mcl_3dl mcl_3dl_lego_map.launch.py
  
  # 指定自定义地图路径
  ros2 launch mcl_3dl mcl_3dl_lego_map.launch.py \
      map_file:=/path/to/custom/map.pcd
  
  # 指定自定义配置文件
  ros2 launch mcl_3dl mcl_3dl_lego_map.launch.py \
      config_file:=/path/to/custom/mcl_config.yaml
  
  # 指定激光话题和里程计话题
  ros2 launch mcl_3dl mcl_3dl_lego_map.launch.py \
      lidar_topic:=/custom/lidar/topic \
      odom_topic:=/custom/odom/topic
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('mcl_3dl')
    
    # ============================================
    # 启动参数声明
    # ============================================
    
    # 地图文件参数 - 默认指向lego_map文件夹
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'dddmr_navigation_ws', 'src', 'lego_map', 'map.pcd'
        ),
        description='Path to the main PCD map file'
    )
    
    # 地面点云文件参数 - 默认使用lego_map中的ground.pcd
    ground_file_arg = DeclareLaunchArgument(
        'ground_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'dddmr_navigation_ws', 'src', 'lego_map', 'ground.pcd'
        ),
        description='Path to the ground PCD file (optional)'
    )
    
    # 子地图目录参数 - 用于多帧地图
    pose_graph_dir_arg = DeclareLaunchArgument(
        'pose_graph_dir',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'dddmr_navigation_ws', 'src', 'lego_map', 'pcd'
        ),
        description='Path to the pose graph (sub-maps) directory'
    )
    
    # MCL配置文件参数
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_share, 'config', 'mcl_3dl_lego_map.yaml'),
        description='Path to the MCL 3DL config file'
    )
    
    # 激光话题参数
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar/pointcloud',
        description='Topic name for LiDAR point cloud'
    )
    
    # 里程计话题参数
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/filtered',
        description='Topic name for odometry'
    )
    
    # 地图下采样参数
    map_downsample_arg = DeclareLaunchArgument(
        'map_downsample',
        default_value='0.2',
        description='Map downsampling resolution (meters)'
    )
    
    # 地面下采样参数
    ground_downsample_arg = DeclareLaunchArgument(
        'ground_downsample',
        default_value='0.2',
        description='Ground downsampling resolution (meters)'
    )
    
    # 启用TF发布参数
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF transform'
    )

    # ============================================
    # 节点1: PCL发布器 (加载地图)
    # ============================================
    pcl_publisher_node = Node(
        package='mcl_3dl',
        executable='pcl_publisher',
        name='pcl_publisher',
        output='screen',
        parameters=[{
            'map_dir': LaunchConfiguration('map_file'),
            'ground_dir': LaunchConfiguration('ground_file'),
            'global_frame': 'map',
            'map_down_sample': LaunchConfiguration('map_downsample'),
            'ground_down_sample': LaunchConfiguration('ground_downsample'),
        }]
    )

    # ============================================
    # 节点2: MCL 3DL定位 (核心定位算法)
    # ============================================
    mcl_3dl_node = Node(
        package='mcl_3dl',
        executable='mcl_3dl',
        name='mcl_3dl',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            ('odom', LaunchConfiguration('odom_topic')),
            ('cloud', LaunchConfiguration('lidar_topic')),
        ]
    )

    # ============================================
    # 返回LaunchDescription
    # ============================================
    return LaunchDescription([
        # 参数声明
        map_file_arg,
        ground_file_arg,
        pose_graph_dir_arg,
        config_file_arg,
        lidar_topic_arg,
        odom_topic_arg,
        map_downsample_arg,
        ground_downsample_arg,
        publish_tf_arg,
        
        # 节点启动
        pcl_publisher_node,
        mcl_3dl_node,
    ])
