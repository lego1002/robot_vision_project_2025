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
        # TF: imu_link → base_link
        Node(
            package='tf2_ros', 
            executable='static_transform_publisher',
            name='imu_tf_pub',
            arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'imu_link']
        ),
        
        # IMU加速度讀取
        Node(
            package='imu_serial',
            executable='imu_data',
            name='imu_data',
            output='screen'
        ),
        
        # IMU速度位置估算
        # Node(
        #     package='imu_serial',
        #     executable='imu_based_odom',
        #     name='imu_based_odom',
        #     output='screen',
        # ),
        
        Node(
            package='imu_serial',
            executable='walk',
            name='walk',
            output='screen',
        ),

        # EKF 
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_path]
        ),
        
        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])