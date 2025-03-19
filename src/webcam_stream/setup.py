from setuptools import setup

package_name = 'webcam_stream'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lego',
    maintainer_email='lego@example.com',
    description='Webcam streaming for ROS2',
    long_description='This package provides webcam streaming functionality.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = webcam_stream.camera_node:main',
        ],
    },
)

