import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    perception_3d_share = get_package_share_directory('perception_3d')
    mcl_3dl_share = get_package_share_directory('mcl_3dl')
    
    # 默认地图路径 (指向 lego_map 文件夹)
    default_map_path = os.path.join(
        os.path.expanduser('~'),
        'dddmr_navigation_ws', 'src', 'lego_map', 'map.pcd'
    )
    default_ground_path = os.path.join(
        os.path.expanduser('~'),
        'dddmr_navigation_ws', 'src', 'lego_map', 'ground.pcd'
    )

    # 1. PCL Publisher - 负责加载并发布 PCD 地图
    pcl_publisher_node = Node(
        package='mcl_3dl',
        executable='pcl_publisher',
        name='pcl_publisher',
        output='screen',
        parameters=[{
            'map_dir': default_map_path,
            'ground_dir': default_ground_path,
            'global_frame': 'map',
            'map_down_sample': 0.1,
            'ground_down_sample': 0.1,
        }]
    )

    # 2. Perception 3D Node - 核心感知节点
    perception_3d_node = Node(
        package='perception_3d',
        executable='perception_3d_ros_node',
        name='perception_3d_test',
        output='screen',
        parameters=[
            os.path.join(perception_3d_share, 'config', 'perception_3d_test.yaml')
        ]
    )

    # 3. 静态 TF (如果需要测试独立效果，通常需要 map -> base_link 的变换)
    # 这里发布一个恒等变换，方便在没有定位的情况下查看地图
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_base',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link']
    )

    return LaunchDescription([
        pcl_publisher_node,
        perception_3d_node,
        #static_tf_node
    ])
