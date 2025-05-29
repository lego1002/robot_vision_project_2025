from setuptools import setup
import os
from glob import glob

package_name = 'imu_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'setuptools',
        'ahrs',
        'numpy',
        'rich'
    ],
    zip_safe=True,
    maintainer='jacky',
    maintainer_email='jackylee20030214@gmail.com',
    description='Serial to IMU ROS2 node',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_data = imu_serial.imu_data:main',
            'imu_based_odom = imu_serial.imu_based_odom:main',
        ],
    },
)