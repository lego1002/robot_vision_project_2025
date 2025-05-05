from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('imu_serial'),
        'config',
        'imu_ekf.yaml'
    )
    
    rviz_config_path = os.path.join(
        get_package_share_directory('imu_serial'),
        'rviz',
        'imu_ekf.rviz'
    )
    
    return LaunchDescription([
        # 靜態 TF: imu_link → base_link（讓 EKF 能正常運作）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_pub',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'imu_link']
        ),
        
        # 啟動node: IMU加速度讀取
        Node(
            package='imu_serial',
            executable='imu_serial_node',
            name='imu_serial_node',
            output='screen'
        ),
        
        # 啟動node: IMU速度估算
        Node(
            package='imu_serial',
            executable='imu_velocity_node',
            name='imu_velocity_node',
            output='screen'
        ),

        # 啟動node: 內建EKF 
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path]
        ),
        
        # 啟動 RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])