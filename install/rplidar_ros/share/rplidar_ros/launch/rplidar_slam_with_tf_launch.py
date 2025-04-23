#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')

    rviz_config = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'rviz',
        'rplidar_ros.rviz'
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port,
                              description='Serial port for RPLIDAR'),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate,
                              description='Baudrate for RPLIDAR'),
        DeclareLaunchArgument('frame_id', default_value=frame_id,
                              description='Frame ID for LIDAR'),
        DeclareLaunchArgument('scan_mode', default_value=scan_mode,
                              description='Scan mode for RPLIDAR'),

        # RPLIDAR 節點
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': scan_mode,
            }],
            output='screen'
        ),

        # 靜態 TF：base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_tf_pub',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        ),
        
        # 靜態 TF： -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_tf_pub',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),

        # SLAM Toolbox 節點（同步模式）
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'odom_frame': 'odom',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'scan_topic': 'scan'
            }]
        ),

        # RViz2 顯示
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
