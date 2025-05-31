from setuptools import setup

package_name = 'my_camera_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lego',
    maintainer_email='lego@example.com',
    description='ROS2 WebSocket Camera Subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber = my_camera_pkg.camera_subscriber:main',
            'camera_node = my_camera_pkg.camera_node:main',
            'canny_edges = my_camera_pkg.canny_edges:main',
            'hsv = my_camera_pkg.hsv:main',
            'hsv_tuner = my_camera_pkg.hsv_tuner:main',
            'object_detect = my_camera_pkg.object_detect:main',
            'hough_circle = my_camera_pkg.hough_circle:main',
            'hough_circle_tuner = my_camera_pkg.hough_circle_tuner:main',
            'yolo = my_camera_pkg.yolo:main',
        ],
    },
)

