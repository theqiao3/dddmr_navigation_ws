import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition

def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('lego_loam_bor')
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'loam_ackermann_mid360_config.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'lego_loam.rviz')

    # 声明启动参数，允许用户自定义话题名称
    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar/pointcloud',
        description='点云话题名称 (默认: /livox/lidar, 备选: /livox/lidar/pointcloud 或其他)'
    )
    
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/filtered',
        description='里程计话题名称 (默认: /odom)'
    )
    
    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu',
        description='IMU话题名称 (默认: /imu/data)'
    )

    # LeGO-LOAM 内部 ImageProjection 会 lookupTransform(laser.baselink_frame, cloud.header.frame_id)
    # 如果系统没有发布这条 TF，可在本launch里可选发布 baselink->sensor 静态TF
    declare_baselink_frame = DeclareLaunchArgument(
        'baselink_frame',
        default_value='base_footprint',
        description='机器人基坐标系（建议与配置 laser.baselink_frame 一致）'
    )
    declare_sensor_frame = DeclareLaunchArgument(
        'sensor_frame',
        default_value='livox_frame',
        description='雷达坐标系（必须等于点云header.frame_id，例如 livox_frame / laser / mid360）'
    )
    declare_publish_static_tf = DeclareLaunchArgument(
        'publish_baselink_to_sensor_tf',
        default_value='false',
        description='是否在本launch内发布 baselink_frame->sensor_frame 的静态TF（系统已发布则保持false）'
    )
    declare_tf_x = DeclareLaunchArgument('tf_x', default_value='0.0', description='静态TF平移x(米)')
    declare_tf_y = DeclareLaunchArgument('tf_y', default_value='0.0', description='静态TF平移y(米)')
    declare_tf_z = DeclareLaunchArgument('tf_z', default_value='0.0', description='静态TF平移z(米)')
    declare_tf_roll = DeclareLaunchArgument('tf_roll', default_value='0.0', description='静态TF roll(弧度)')
    declare_tf_pitch = DeclareLaunchArgument('tf_pitch', default_value='0.0', description='静态TF pitch(弧度)')
    declare_tf_yaw = DeclareLaunchArgument('tf_yaw', default_value='0.0', description='静态TF yaw(弧度)')

    lidar_topic = LaunchConfiguration('lidar_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    baselink_frame = LaunchConfiguration('baselink_frame')
    sensor_frame = LaunchConfiguration('sensor_frame')

    # LeGO-LOAM 节点
    lego_loam_node = Node(
        package='lego_loam_bor',
        executable='lego_loam',
        # 注意：该可执行文件内部会创建多个 rclcpp::Node（lego_loam_ip / lego_loam_fa / lego_loam_mo ...）。
        # 在 launch 里强行设置 name 会通过 __node 重映射让这些节点全变成同一个名字，导致参数文件匹配失败。
        output='screen',
        respawn=False,
        parameters=[config_file],
        remappings=[
            # 源码中订阅话题名是相对名（无前导/），这里也用相对名做 remap 才能正确生效
            ('lslidar_point_cloud', lidar_topic),
            ('odom', odom_topic),
            ('imu/data', imu_topic)
        ]
    )

    # RViz 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        respawn=False,
        arguments=['-d', rviz_config]
    )

    tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='baselink_to_sensor_static_tf',
        output='screen',
        condition=IfCondition(LaunchConfiguration('publish_baselink_to_sensor_tf')),
        arguments=[
            '--x', LaunchConfiguration('tf_x'),
            '--y', LaunchConfiguration('tf_y'),
            '--z', LaunchConfiguration('tf_z'),
            '--roll', LaunchConfiguration('tf_roll'),
            '--pitch', LaunchConfiguration('tf_pitch'),
            '--yaw', LaunchConfiguration('tf_yaw'),
            '--frame-id', baselink_frame,
            '--child-frame-id', sensor_frame,
        ],
    )

    return LaunchDescription([
        declare_lidar_topic,
        declare_odom_topic,
        declare_imu_topic,
        declare_baselink_frame,
        declare_sensor_frame,
        declare_publish_static_tf,
        declare_tf_x,
        declare_tf_y,
        declare_tf_z,
        declare_tf_roll,
        declare_tf_pitch,
        declare_tf_yaw,
        lego_loam_node,
        rviz_node,
        tf_publisher,
    ])
