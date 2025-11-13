from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'smartsort_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), 
            glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), 
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SmartSort Team',
    maintainer_email='smartsort@example.com',
    description='Autonomous Warehouse Sorting Robot with ROS2 Jazzy',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_brain = smartsort_robot.robot_brain:main',
            'gripper_controller = smartsort_robot.gripper_controller:main',
        ],
    },
)
