import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lego_loam_bor')
    config_file = os.path.join(pkg_share, 'config', 'loam_bag_ackermann_mid360_config.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'lego_loam.rviz')

    declare_lidar_topic = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/livox/lidar/pointcloud',
        description='点云话题名称（从 rosbag 中读取）'
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

    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        default_value='/home/tianbot/rosbag2_2025_12_19-13_56_10',
        description='待播放的数据包路径'
    )

    declare_bag_rate = DeclareLaunchArgument(
        'bag_rate',
        default_value='1.0',
        description='播放速率'
    )


    declare_debug_info = DeclareLaunchArgument(
        'debug_info',
        default_value='false',
        description='是否输出调试信息'
    )

    lidar_topic = LaunchConfiguration('lidar_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    imu_topic = LaunchConfiguration('imu_topic')
    baselink_frame = LaunchConfiguration('baselink_frame')
    sensor_frame = LaunchConfiguration('sensor_frame')
    bag_path = LaunchConfiguration('bag_path')
    bag_rate = LaunchConfiguration('bag_rate')
    voxel_size = LaunchConfiguration('voxel_size')
    enable_preprocessor = LaunchConfiguration('enable_preprocessor')
    debug_info = LaunchConfiguration('debug_info')

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

    bag_player = ExecuteProcess(
        cmd=[
            'ros2',
            'bag',
            'play',
            '--rate',
            bag_rate,
            bag_path,
        ],
        output='screen',
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
        declare_bag_path,
        declare_bag_rate,
        declare_debug_info,
        lego_loam_node,
        rviz_node,
        #tf_publisher,
        #cloud_preprocessor_node,
        TimerAction(period=1.0, actions=[bag_player]),
    ])
