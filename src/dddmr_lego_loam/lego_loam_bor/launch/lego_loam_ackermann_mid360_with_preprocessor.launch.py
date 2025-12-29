import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lego_loam_bor')
    config_file = os.path.join(pkg_share, 'config', 'loam_ackermann_mid360_config.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'lego_loam.rviz')

    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar/pointcloud_filtered',
        description='点云话题名称（已滤波）'
    )

    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/filtered',
        description='里程计话题名称'
    )

    declare_imu_topic = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu',
        description='IMU话题名称'
    )

    declare_baselink_frame = DeclareLaunchArgument(
        'baselink_frame',
        default_value='base_link',
        description='机器人基坐标系'
    )

    declare_sensor_frame = DeclareLaunchArgument(
        'sensor_frame',
        default_value='livox_frame',
        description='雷达坐标系'
    )

    declare_publish_static_tf = DeclareLaunchArgument(
        'publish_baselink_to_sensor_tf',
        default_value='false',
        description='是否发布静态TF'
    )

    declare_tf_x = DeclareLaunchArgument('tf_x', default_value='0.0', description='静态TF平移x(米)')
    declare_tf_y = DeclareLaunchArgument('tf_y', default_value='0.0', description='静态TF平移y(米)')
    declare_tf_z = DeclareLaunchArgument('tf_z', default_value='0.0', description='静态TF平移z(米)')
    declare_tf_roll = DeclareLaunchArgument('tf_roll', default_value='0.0', description='静态TF roll(弧度)')
    declare_tf_pitch = DeclareLaunchArgument('tf_pitch', default_value='0.0', description='静态TF pitch(弧度)')
    declare_tf_yaw = DeclareLaunchArgument('tf_yaw', default_value='0.0', description='静态TF yaw(弧度)')

    declare_voxel_size = DeclareLaunchArgument(
        'voxel_size',
        default_value='0.1',
        description='体素大小（米）'
    )

    declare_enable_preprocessor = DeclareLaunchArgument(
        'enable_preprocessor',
        default_value='true',
        description='是否启用点云预处理器'
    )

    declare_debug_info = DeclareLaunchArgument(
        'debug_info',
        default_value='true',
        description='是否输出调试信息'
    )

    lidar_topic = LaunchConfiguration('lidar_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    baselink_frame = LaunchConfiguration('baselink_frame')
    sensor_frame = LaunchConfiguration('sensor_frame')
    voxel_size = LaunchConfiguration('voxel_size')
    enable_preprocessor = LaunchConfiguration('enable_preprocessor')
    debug_info = LaunchConfiguration('debug_info')

    # Cloud preprocessor node
    cloud_preprocessor_node = Node(
        package='lego_loam_bor',
        executable='cloud_preprocessor',
        name='cloud_preprocessor',
        output='screen',
        condition=IfCondition(enable_preprocessor),
        parameters=[{
            'voxel_size': voxel_size,
            'debug_info': debug_info,
            'input_topic': '/livox/lidar/pointcloud',
            'output_topic': '/livox/lidar/pointcloud_filtered'
        }]
    )

    lego_loam_node = Node(
        package='lego_loam_bor',
        executable='lego_loam',
        output='screen',
        respawn=False,
        parameters=[config_file],
        remappings=[
            ('lslidar_point_cloud', lidar_topic),
            ('odom', odom_topic),
            ('imu/data', imu_topic)
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
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
        declare_voxel_size,
        declare_enable_preprocessor,
        declare_debug_info,
        cloud_preprocessor_node,
        lego_loam_node,
        rviz_node,
        tf_publisher,
    ])
