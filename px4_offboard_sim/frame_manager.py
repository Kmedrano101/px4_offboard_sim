#!/usr/bin/env python3

__author__ = "Kevin Medrano Ayala"
__contact__ = "kevin.ejem18@gmail.com"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

class FrameRemapper(Node):
    def __init__(self):
        super().__init__('frame_manager')
        
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribing to sensor topics from Gazebo
        self.camera_sub = self.create_subscription(Image, "/sim_camera/camera", self.camera_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/sim_camera/camera_info", self.camera_info_callback, 10)
        self.depth_camera_sub = self.create_subscription(Image, "/sim_depth_camera/depth_camera", self.depth_camera_callback, 10)
        self.depth_points_sub = self.create_subscription(PointCloud2, "/sim_depth_camera/points", self.depth_points_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/sim_imu/imu", self.imu_callback, 10)

        # Publishers for modified messages with updated frame_ids
        self.camera_pub = self.create_publisher(Image, "/camera", qos_profile)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/camera_info", qos_profile)
        self.depth_camera_pub = self.create_publisher(Image, "/depth_camera", qos_profile)
        self.depth_points_pub = self.create_publisher(PointCloud2, "/depth_camera/points", qos_profile)
        self.imu_pub = self.create_publisher(Imu, "/imu", qos_profile)

    def camera_callback(self, msg):
        """ Modify frame_id for RGB camera """
        msg.header.frame_id = "camera_link"  # Change frame_id
        self.camera_pub.publish(msg)
        self.get_logger().info("Republished RGB Camera with corrected frame_id")

    def camera_info_callback(self, msg):
        """ Modify frame_id for Camera Info """
        msg.header.frame_id = "camera_link"  # Change frame_id
        self.camera_info_pub.publish(msg)
        self.get_logger().info("Republished Camera Info with corrected frame_id")

    def depth_camera_callback(self, msg):
        """ Modify frame_id for Depth camera """
        msg.header.frame_id = "depth_camera_link"  # Change frame_id
        self.depth_camera_pub.publish(msg)
        self.get_logger().info("Republished Depth Camera with corrected frame_id")

    def depth_points_callback(self, msg):
        """ Modify frame_id for PointCloud2 """
        msg.header.frame_id = "depth_camera_link"  # Change frame_id
        self.depth_points_pub.publish(msg)
        self.get_logger().info("Republished Depth Camera Points with corrected frame_id")
    
    def imu_callback(self, msg):
        """ Modify frame_id for IMU data """
        msg.header.frame_id = "imu_link"
        self.imu_pub.publish(msg)
        self.get_logger().info("Republished IMU data with corrected frame_id")

def main(args=None):
    rclpy.init(args=args)
    node = FrameRemapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
