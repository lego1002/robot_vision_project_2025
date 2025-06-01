from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 啟動你的 camera node
        Node(
            package='my_camera_pkg',
            executable='camera_node',
            name='camera_node',
            output='screen'
        ),
        
        # Node(
        #     package='my_camera_pkg',
        #     executable='hsv_tuner',
        #     name='hsv_tuner',
        #     output='screen'
        # ),
        
        Node(
            package='my_camera_pkg',
            executable='hsv',
            name='hsv',
            output='screen'
        ),
        
        # --------------------------------
        Node(
            package='my_camera_pkg',
            executable='canny_edges',
            name='canny_edges',
            output='screen'
        ),
        
        # Node(
        #     package='my_camera_pkg',
        #     executable='object_detect',
        #     name='object_detect',
        #     output='screen'
        # ),
        
        # --------------------------------
        Node(
            package='my_camera_pkg',
            executable='hough_circle',
            name='hough_circle',
            output='screen'
        ),
        
        # Node(
        #     package='my_camera_pkg',
        #     executable='hough_circle_tuner',
        #     name='hough_circle_tuner',
        #     output='screen'
        # ),
        
        Node(
            package='my_camera_pkg',
            executable='yolo',
            name='yolo',
            output='screen'
        ),
        
        Node(
            package='my_camera_pkg',
            executable='tkinter',
            name='tkinter',
            output='screen'
        ),
        
        Node(
            package='my_camera_pkg',
            executable='hole_template_match',
            name='hole_template_match',
            output='screen'
        ),
        
        # 啟動 rqt_image_view
        # Node(
        #     package='rqt_image_view',
        #     executable='rqt_image_view',
        #     name='rqt_image_view',
        #     output='screen'
        # ),
    ])
