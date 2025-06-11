#!/usr/bin/env python3

__author__ = "Kevin Medrano Ayala"
__contact__ = "kevin.ejem18@gmail.com"

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard_sim')
    
    return LaunchDescription([
        #SetParameter(name='use_sim_time', value=True),
        # ExecuteProcess(cmd=['bash', bash_script_path], output='screen'),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            namespace='px4_offboard_sim',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='px4_offboard_sim',
            namespace='px4_offboard_sim',
            executable='joy_control',
            name='joy_control',
            prefix='gnome-terminal --',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_parameter_bridge',
            output='screen',
            parameters=[
                {'config_file': os.path.join(package_dir, 'config', 'gz_bridge.yaml')},
                {'use_sim_time': True}
            ],
        ),
        Node(
            package='px4_offboard_sim',
            namespace='px4_offboard_sim',
            executable='processes',
            name='processes',
            prefix='gnome-terminal --',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='px4_offboard_sim',
            executable='frame_manager',
            name='frame_manager',
            namespace='px4_offboard_sim',
            prefix='gnome-terminal --',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='px4_offboard_sim',
            namespace='px4_offboard_sim',
            executable='offboard_control',
            name='offboard_control',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])