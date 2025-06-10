import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import std_msgs.msg
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import time
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class AutoNavigation(Node):
    def __init__(self):
        super().__init__('auto_navigation')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.arm_pub = self.create_publisher(std_msgs.msg.Bool, '/arm_message', qos_profile)
        self.cmd_pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_profile)
        self.depth_sub = self.create_subscription(PointCloud2, '/depth_camera/points', self.depth_callback, qos_profile)
        self.arm_toggle = False
        self.obstacle_detected = False

    def depth_callback(self, msg):
        # Process the point cloud data to detect obstacles
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        for point in points:
            x, y, z = point
            if -0.5 < y < 0.5 and 0 < x < 2.0 and z < 2.0:  # Adjust the range as needed
                self.obstacle_detected = True
                self.get_logger().info('Obstacle detected')
                break

    def arm_drone(self):
        self.arm_toggle = True
        arm_msg = std_msgs.msg.Bool()
        arm_msg.data = self.arm_toggle
        self.arm_pub.publish(arm_msg)
        self.get_logger().info('Drone armed')

    def move_forward(self, speed):
        twist = Twist()
        twist.linear.y = speed

        while not self.obstacle_detected:
            self.cmd_pub.publish(twist)
            time.sleep(0.1)

        # Stop the drone when an obstacle is detected
        twist.linear.y = 0.0
        self.cmd_pub.publish(twist)
        self.get_logger().info('Drone stopped due to obstacle')

def main(args=None):
    rclpy.init(args=args)
    auto_nav = AutoNavigation()

    # Wait for Enter key press to start auto navigation
    input("Press Enter to start auto navigation...")

    # Arm the drone
    auto_nav.arm_drone()
    time.sleep(2)  # Wait for 2 seconds to ensure the drone is armed

    # Move the drone forward at a speed of 1 meter per second
    auto_nav.move_forward(speed=1.0)

    auto_nav.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
