"""Launch joy_node + joy_control_node with controller selection.

Usage:
  ros2 launch px4_offboard_sim joy_control.launch.py                      # T-Pro (default)
  ros2 launch px4_offboard_sim joy_control.launch.py controller:=tpro
  ros2 launch px4_offboard_sim joy_control.launch.py controller:=gamepad
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os

CONTROLLERS = {
    'tpro': {
        'config': 'joy_control_tpro.yaml',
        'joy_dev': '/dev/input/js1',
        'deadzone': 0.03,
    },
    'gamepad': {
        'config': 'joy_control.yaml',
        'joy_dev': '/dev/input/js0',
        'deadzone': 0.05,
    },
}


def _launch_nodes(context):
    controller = LaunchConfiguration('controller').perform(context)
    joy_dev = LaunchConfiguration('joy_dev').perform(context)

    if controller not in CONTROLLERS:
        raise ValueError(
            f"Unknown controller '{controller}'. Choose from: {list(CONTROLLERS.keys())}")

    cfg = CONTROLLERS[controller]

    if not joy_dev:
        joy_dev = cfg['joy_dev']

    pkg_share = FindPackageShare('px4_offboard_sim').perform(context)
    config_file = os.path.join(pkg_share, 'config', cfg['config'])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'device_id': 0,
            'deadzone': cfg['deadzone'],
            'autorepeat_rate': 20.0,
            'dev': joy_dev,
        }],
        output='screen',
    )

    joy_control_node = Node(
        package='px4_offboard_sim',
        executable='joy_control_node',
        name='joy_control',
        parameters=[config_file],
        output='screen',
    )

    return [joy_node, joy_control_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'controller', default_value='tpro',
            description='Controller type: tpro, gamepad',
        ),
        DeclareLaunchArgument(
            'joy_dev', default_value='',
            description='Joystick device path override (e.g. /dev/input/js1)',
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
