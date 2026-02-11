from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('px4_offboard_sim'), 'config', 'offboard_control.yaml'
    ])

    offboard_node = Node(
        package='px4_offboard_sim',
        executable='offboard_control_node',
        name='offboard_control',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([offboard_node])
