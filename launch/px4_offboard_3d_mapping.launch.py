from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo principal de RTAB-Map
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            parameters=[{
                "frame_id": "base_link",
                "subscribe_depth": True,
                "subscribe_odom": True,
                "approx_sync": True,
                "queue_size": 20,
                "RGBD/NeighborLinkRefining": "true",
            }],
            remappings=[
                ("rgb/image", "/camera/color/image_raw"),
                ("depth/image", "/camera/depth/image_raw"),
                ("rgb/camera_info", "/camera/color/camera_info"),
                ("odom", "/odom"),
            ],
            output="screen",
            name='rtabmap_node'
        ),
        
        # Publicador estático de TF entre base_link y camera_link
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.1", "0", "0.2", "0", "0", "0", "base_link", "camera_link"],
            name="camera_tf_publisher"
        )
    ])