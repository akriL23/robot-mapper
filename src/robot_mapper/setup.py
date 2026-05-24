from setuptools import setup
import os
from glob import glob

package_name = 'robot_mapper'

setup(
    name=package_name,
    version='0.0.2',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'), 
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'flask', 'flask-cors', 'pyserial'],
    zip_safe=True,
    maintainer='akril',
    maintainer_email='akril@example.com',
    description='ROS2 robot mapper for Raspberry Pi + L298N + YDLidar X4 + US sensors',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_ros_node = robot_mapper.robot_ros_node:main',
            'web_control = robot_mapper.web_control:main',
            'auto_explorer = robot_mapper.auto_explorer:main',
            'cmd_vel_mux = robot_mapper.cmd_vel_mux:main',
            'camera_publisher = robot_mapper.camera_publisher:main',
            'joystick_control = robot_mapper.joystick_control:main',
            'ultrasonic_node = robot_mapper.ultrasonic_node:main',
        ],
    },
)
