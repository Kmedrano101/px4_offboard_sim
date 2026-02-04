import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.yaml')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'docs'), glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kmedrano101',
    maintainer_email='kevin.ejem18@gmail.com',
    description='ROS2 Jazzy PX4 Offboard Simulation Package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'offboard_control = px4_offboard_sim.offboard_control:main',
            'joy_control = px4_offboard_sim.joy_control:main',
            'processes = px4_offboard_sim.processes:main',
        ],
    },
)
