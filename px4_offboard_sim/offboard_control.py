#!/usr/bin/env python3

__author__ = "Kevin Medrano Ayala"
__contact__ = "kevin.ejem18@gmail.com"

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import VehicleAttitude
from px4_msgs.msg import VehicleCommand
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, Imu
from std_msgs.msg import Bool


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control')

        # QoS profile for PX4 v1.17+ compatibility
        # PX4 uses BEST_EFFORT reliability and VOLATILE durability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS profile for joy_control communication (matches joy_control publisher)
        joy_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create subscriptions
        # Note: PX4 v1.17 uses vehicle_status_v1 topic
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile)

        self.offboard_velocity_sub = self.create_subscription(
            Twist,
            '/px4_offboard_sim/offboard_velocity_cmd',
            self.offboard_velocity_callback,
            joy_qos_profile)

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            qos_profile)

        self.my_bool_sub = self.create_subscription(
            Bool,
            '/px4_offboard_sim/arm_message',
            self.arm_message_callback,
            joy_qos_profile)

        # Frame remapping subscriptions (Gazebo sensor topics)
        self.camera_sub = self.create_subscription(Image, "/sim_camera/camera", self.camera_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/sim_camera/camera_info", self.camera_info_callback, 10)
        self.depth_camera_sub = self.create_subscription(Image, "/sim_depth_camera/depth_camera", self.depth_camera_callback, 10)
        self.depth_points_sub = self.create_subscription(PointCloud2, "/sim_depth_camera/points", self.depth_points_callback, 10)
        self.imu_sub = self.create_subscription(Imu, "/sim_imu/imu", self.imu_callback, 10)

        # Create publishers (all use same QoS for PX4 compatibility)
        self.publisher_offboard_mode = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.publisher_trajectory = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher_ = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        # Frame remapping publishers (with corrected frame_ids)
        # Use default QoS for sensor data (different from PX4 topics)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.camera_pub = self.create_publisher(Image, "/camera", sensor_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/camera_info", sensor_qos)
        self.depth_camera_pub = self.create_publisher(Image, "/depth_camera", sensor_qos)
        self.depth_points_pub = self.create_publisher(PointCloud2, "/depth_camera/points", sensor_qos)
        self.imu_pub = self.create_publisher(Imu, "/imu", sensor_qos)

        # Main control loop timer - runs at 50Hz (must be > 2Hz for PX4)
        timer_period = 0.02  # 50Hz
        self.timer = self.create_timer(timer_period, self.control_loop_callback)

        # State variables
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_DISARMED
        self.velocity = Vector3()
        self.yaw = 0.0
        self.trueYaw = 0.0
        self.flightCheck = False
        self.failsafe = False
        self.arm_message = False

        # State machine
        self.current_state = "IDLE"
        self.last_state = ""
        self.offboard_setpoint_counter = 0

        # Required streaming time before arming (1 second at 50Hz = 50 cycles)
        self.STREAMING_THRESHOLD = 50

        self.get_logger().info("Offboard Control Node initialized")
        self.get_logger().info("Waiting for arm command...")

    def arm_message_callback(self, msg):
        self.arm_message = msg.data
        self.get_logger().info(f"Arm command received: {self.arm_message}")

    def control_loop_callback(self):
        """
        Main control loop - handles state machine and publishes control messages.
        PX4 v1.17 requires:
        1. Stream OffboardControlMode + TrajectorySetpoint BEFORE arming
        2. Minimum 2Hz rate, must stream for at least 1 second before arming
        3. Enter offboard mode BEFORE arming
        """

        # Always publish offboard control mode and setpoints (required by PX4)
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()

        # State machine for arming sequence
        match self.current_state:
            case "IDLE":
                self.offboard_setpoint_counter = 0
                if self.arm_message and self.flightCheck:
                    self.current_state = "STREAMING"
                    self.get_logger().info("Starting setpoint streaming...")

            case "STREAMING":
                # Stream setpoints for at least 1 second before switching to offboard
                self.offboard_setpoint_counter += 1

                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Flight check failed, returning to IDLE")
                elif not self.arm_message:
                    self.current_state = "IDLE"
                    self.get_logger().info("Arm command cancelled")
                elif self.offboard_setpoint_counter >= self.STREAMING_THRESHOLD:
                    self.current_state = "SWITCH_TO_OFFBOARD"
                    self.get_logger().info(f"Streamed {self.offboard_setpoint_counter} setpoints, switching to offboard mode...")

            case "SWITCH_TO_OFFBOARD":
                if not self.flightCheck or not self.arm_message:
                    self.current_state = "IDLE"
                    self.get_logger().info("Aborting, returning to IDLE")
                else:
                    # Send offboard mode command
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                        param1=1.0,  # Custom mode
                        param2=6.0   # Offboard mode
                    )
                    # Check if we're in offboard mode
                    if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                        self.current_state = "ARMING"
                        self.get_logger().info("Offboard mode active, arming...")

            case "ARMING":
                if not self.flightCheck:
                    self.current_state = "IDLE"
                    self.get_logger().info("Flight check failed during arming")
                elif self.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.current_state = "ARMED"
                    self.get_logger().info("Armed successfully! Vehicle is now in offboard control.")
                else:
                    # Send arm command
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        param1=1.0  # 1.0 = arm
                    )

            case "ARMED":
                # Check for disarm or failsafe conditions
                if self.arm_state != VehicleStatus.ARMING_STATE_ARMED:
                    self.current_state = "IDLE"
                    self.arm_message = False
                    self.get_logger().info("Disarmed, returning to IDLE")
                elif self.failsafe:
                    self.current_state = "IDLE"
                    self.arm_message = False
                    self.get_logger().info("Failsafe triggered, returning to IDLE")
                elif not self.arm_message:
                    # Disarm if arm_message is set to false
                    self.publish_vehicle_command(
                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        param1=0.0  # 0.0 = disarm
                    )
                    self.get_logger().info("Disarm command sent")

        # Log state changes
        if self.last_state != self.current_state:
            self.last_state = self.current_state

    def publish_offboard_control_mode(self):
        """Publish offboard control mode - must be sent continuously at > 2Hz"""
        msg = OffboardControlMode()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.publisher_offboard_mode.publish(msg)

    def publish_trajectory_setpoint(self):
        """Publish trajectory setpoint with velocity commands"""
        # Compute velocity in the world frame
        cos_yaw = np.cos(self.trueYaw)
        sin_yaw = np.sin(self.trueYaw)
        velocity_world_x = (self.velocity.x * cos_yaw - self.velocity.y * sin_yaw)
        velocity_world_y = (self.velocity.x * sin_yaw + self.velocity.y * cos_yaw)

        msg = TrajectorySetpoint()
        msg.timestamp = int(Clock().now().nanoseconds / 1000)

        # Velocity setpoints (NED frame)
        msg.velocity[0] = velocity_world_x
        msg.velocity[1] = velocity_world_y
        msg.velocity[2] = self.velocity.z

        # Position set to NaN (not used in velocity control)
        msg.position[0] = float('nan')
        msg.position[1] = float('nan')
        msg.position[2] = float('nan')

        # Acceleration set to NaN
        msg.acceleration[0] = float('nan')
        msg.acceleration[1] = float('nan')
        msg.acceleration[2] = float('nan')

        # Yaw control
        msg.yaw = float('nan')
        msg.yawspeed = self.yaw

        self.publisher_trajectory.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """Publish a vehicle command to PX4"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(Clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher_.publish(msg)

    def vehicle_status_callback(self, msg):
        """Receive and process vehicle status updates"""
        if msg.nav_state != self.nav_state:
            self.get_logger().info(f"NAV_STATE: {msg.nav_state}")

        if msg.arming_state != self.arm_state:
            self.get_logger().info(f"ARM_STATE: {msg.arming_state}")

        if msg.failsafe != self.failsafe:
            self.get_logger().info(f"FAILSAFE: {msg.failsafe}")

        if msg.pre_flight_checks_pass != self.flightCheck:
            self.get_logger().info(f"FLIGHT_CHECK: {msg.pre_flight_checks_pass}")

        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe = msg.failsafe
        self.flightCheck = msg.pre_flight_checks_pass

    def offboard_velocity_callback(self, msg):
        """Receive velocity commands and convert from FLU to NED"""
        # FLU (Front-Left-Up) to NED (North-East-Down) transformation
        self.velocity.x = -msg.linear.y   # X (NED) = -Y (FLU)
        self.velocity.y = msg.linear.x    # Y (NED) = X (FLU)
        self.velocity.z = -msg.linear.z   # Z (NED) = -Z (FLU)
        self.yaw = msg.angular.z

    def attitude_callback(self, msg):
        """Receive vehicle attitude and extract yaw"""
        orientation_q = msg.q
        self.trueYaw = -(np.arctan2(
            2.0 * (orientation_q[3] * orientation_q[0] + orientation_q[1] * orientation_q[2]),
            1.0 - 2.0 * (orientation_q[0] * orientation_q[0] + orientation_q[1] * orientation_q[1])
        ))

    # Frame remapping callbacks
    def camera_callback(self, msg):
        msg.header.frame_id = "camera_link"
        self.camera_pub.publish(msg)

    def camera_info_callback(self, msg):
        msg.header.frame_id = "camera_link"
        self.camera_info_pub.publish(msg)

    def depth_camera_callback(self, msg):
        msg.header.frame_id = "depth_camera_link"
        self.depth_camera_pub.publish(msg)

    def depth_points_callback(self, msg):
        msg.header.frame_id = "depth_camera_link"
        self.depth_points_pub.publish(msg)

    def imu_callback(self, msg):
        msg.header.frame_id = "imu_link"
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
