import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard_sim'

setup(
    name=package_name.replace('_', '-'),
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/ament_index/resource_index/packages',
            ['config/' + 'gz_bridge.yaml']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
        # (os.path.join('share', package_name), ['scripts/TerminatorScript.sh'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kmedrano101',
    maintainer_email='kevin.ejem18@gmail.com',
    description='OFFBOARD SIMULATION FOR PX4 DRONE USING ROS2',
    license='MIT License',
    #tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'offboard_control = px4_offboard_sim.offboard_control:main',
                'joy_control = px4_offboard_sim.joy_control:main',
                'frame_manager = px4_offboard_sim.frame_manager:main',
                'processes = px4_offboard_sim.processes:main'
        ],
    },
)
