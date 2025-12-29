import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        default_value='/livox/lidar/pointcloud',
        description='输入点云话题'
    )

    declare_output_topic = DeclareLaunchArgument(
        'output_topic',
        default_value='/livox/lidar/pointcloud_filtered',
        description='输出点云话题'
    )

    declare_voxel_size = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.1',
        description='体素滤波的立方体边长（米）'
    )

    declare_debug_info = DeclareLaunchArgument(
        'debug_info',
        default_value='true',
        description='是否输出调试信息'
    )

    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    voxel_size = LaunchConfiguration('voxel_size')
    debug_info = LaunchConfiguration('debug_info')

    cloud_preprocessor_node = Node(
        package='lego_loam_bor',
        executable='cloud_preprocessor',
        name='cloud_preprocessor',
        output='screen',
        parameters=[{
            'input_topic': input_topic,
            'output_topic': output_topic,
            'voxel_size': voxel_size,
            'debug_info': debug_info
        }]
    )

    return LaunchDescription([
        declare_input_topic,
        declare_output_topic,
        declare_voxel_size,
        declare_debug_info,
        cloud_preprocessor_node,
    ])
