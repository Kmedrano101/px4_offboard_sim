# Sensor Configuration and Gazebo-ROS2 Bridge

This document describes the sensor setup for the `x500_lidar_360` drone model in the `baylands_custom` Gazebo world, including extrinsic calibration values and topic bridging details.

## Table of Contents

- [Coordinate Frames](#coordinate-frames)
- [Sensor Extrinsics](#sensor-extrinsics)
- [Gazebo-ROS2 Bridge](#gazebo-ros2-bridge)
- [Topic Reference](#topic-reference)
- [Usage](#usage)

---

## Coordinate Frames

### TF Tree Structure

```
map
 └── odom
      └── base_link          (drone body frame)
           └── lidar_link    (LiDAR sensor frame)
```

### Frame Definitions

| Frame | Description |
|-------|-------------|
| `map` | World-fixed frame (ENU orientation) |
| `odom` | Odometry frame, may drift over time |
| `base_link` | Drone body center, IMU location |
| `lidar_link` | LiDAR sensor optical frame |

### Coordinate System Convention

All frames follow **REP-103** (ROS Enhancement Proposal):
- **X**: Forward
- **Y**: Left
- **Z**: Up

---

## Sensor Extrinsics

### IMU (base_link)

The IMU is located at the drone's center of mass.

| Parameter | Value |
|-----------|-------|
| Position | `(0, 0, 0)` m relative to base_link |
| Orientation | Identity (aligned with base_link) |
| Update Rate | 200 Hz |

### LiDAR (lidar_link)

The 3D LiDAR is mounted on top center of the drone.

| Parameter | Value |
|-----------|-------|
| Position | `(0, 0, 0.26)` m relative to base_link |
| Orientation | Identity (aligned with base_link) |
| Update Rate | 20 Hz |
| Horizontal FOV | 360° (-180° to +180°) |
| Horizontal Samples | 720 (0.5° resolution) |
| Vertical FOV | 45° (-22.5° to +22.5°) |
| Vertical Samples | 16 channels (2.8° resolution) |
| Points per Scan | 11,520 (720 x 16) |
| Range | 0.1 - 30.0 m |

### Transform: base_link → lidar_link

```yaml
# Static transform (no rotation, only Z offset)
translation:
  x: 0.0
  y: 0.0
  z: 0.26

rotation (quaternion):
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0

rotation (RPY radians):
  roll:  0.0
  pitch: 0.0
  yaw:   0.0
```

### Publishing Static Transform

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.26 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id lidar_link
```

Or in a launch file:

```python
from launch_ros.actions import Node

Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    arguments=['0', '0', '0.26', '0', '0', '0', 'base_link', 'lidar_link'],
)
```

---

## Gazebo-ROS2 Bridge

### Overview

The `ros_gz_bridge` package translates messages between Gazebo Transport and ROS2 DDS. Each sensor in Gazebo publishes to a Gazebo topic, which the bridge converts to ROS2 messages.

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         GAZEBO SIMULATION                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌────────────┐  │
│  │ IMU Sensor  │  │ LiDAR 360   │  │ Magnetometer│  │    GPS     │  │
│  │  (200 Hz)   │  │  (20 Hz)    │  │  (100 Hz)   │  │  (30 Hz)   │  │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └─────┬──────┘  │
│         │                │                │                │        │
│         ▼                ▼                ▼                ▼        │
│  gz.msgs.IMU    gz.msgs.PointCloudPacked  gz.msgs.Mag   gz.msgs.NavSat │
└─────────┬────────────────┬────────────────┬────────────────┬───────────┘
          │                │                │                │
          │    Gazebo Transport Topics      │                │
          │                │                │                │
┌─────────▼────────────────▼────────────────▼────────────────▼───────────┐
│                       ros_gz_bridge                                    │
│                   (parameter_bridge node)                              │
│                                                                        │
│  Config: config/gz_bridge.yaml                                         │
└─────────┬────────────────┬────────────────┬────────────────┬───────────┘
          │                │                │                │
          │      ROS2 DDS Topics            │                │
          │                │                │                │
          ▼                ▼                ▼                ▼
   sensor_msgs/Imu  sensor_msgs/   sensor_msgs/    sensor_msgs/
                    PointCloud2    MagneticField   NavSatFix
```

### Bridge Configuration

The bridge is configured via YAML file: `config/gz_bridge.yaml`

```yaml
# Example entry
- gz_topic_name: "/world/baylands_custom/model/x500_lidar_360/link/base_link/sensor/imu_sensor/imu"
  ros_topic_name: "/imu/data"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: "GZ_TO_ROS"
```

#### Configuration Parameters

| Parameter | Description |
|-----------|-------------|
| `gz_topic_name` | Full Gazebo topic path |
| `ros_topic_name` | Desired ROS2 topic name |
| `gz_type_name` | Gazebo message type |
| `ros_type_name` | ROS2 message type |
| `direction` | `GZ_TO_ROS`, `ROS_TO_GZ`, or `BIDIRECTIONAL` |
| `publisher_queue` | ROS2 publisher queue size (default: 10) |
| `subscriber_queue` | Gazebo subscriber queue size (default: 10) |

---

## Topic Reference

### Gazebo Topics (Source)

| Sensor | Gazebo Topic |
|--------|--------------|
| IMU | `/world/baylands_custom/model/x500_lidar_360/link/base_link/sensor/imu_sensor/imu` |
| LiDAR (PointCloud) | `/world/baylands_custom/model/x500_lidar_360/link/lidar_link/sensor/lidar_360/scan/points` |
| LiDAR (LaserScan) | `/world/baylands_custom/model/x500_lidar_360/link/lidar_link/sensor/lidar_360/scan` |
| Magnetometer | `/world/baylands_custom/model/x500_lidar_360/link/base_link/sensor/magnetometer_sensor/magnetometer` |
| Barometer | `/world/baylands_custom/model/x500_lidar_360/link/base_link/sensor/air_pressure_sensor/air_pressure` |
| GPS | `/world/baylands_custom/model/x500_lidar_360/link/base_link/sensor/navsat_sensor/navsat` |
| Clock | `/clock` |

### ROS2 Topics (Bridged)

| Topic | Message Type | Rate | Frame ID | Use Case |
|-------|--------------|------|----------|----------|
| `/imu/data` | `sensor_msgs/msg/Imu` | 200 Hz | `base_link` | State estimation |
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | 20 Hz | `lidar_link` | SLAM, 3D mapping |
| `/lidar/scan` | `sensor_msgs/msg/LaserScan` | 20 Hz | `lidar_link` | Nav2, 2D visualization |
| `/mag/data` | `sensor_msgs/msg/MagneticField` | 100 Hz | `base_link` | Heading estimation |
| `/baro/data` | `sensor_msgs/msg/FluidPressure` | 50 Hz | `base_link` | Altitude estimation |
| `/gps/fix` | `sensor_msgs/msg/NavSatFix` | 30 Hz | `base_link` | Global positioning |
| `/clock` | `rosgraph_msgs/msg/Clock` | sim time | - | Time sync |

### Message Details

#### IMU (`/imu/data`)

```
sensor_msgs/msg/Imu
├── header
│   ├── stamp        # Simulation timestamp
│   └── frame_id     # "base_link"
├── orientation      # Quaternion (x, y, z, w)
├── angular_velocity # rad/s (x, y, z)
└── linear_acceleration # m/s² (x, y, z)
```

#### PointCloud2 (`/lidar/points`) - Primary for SLAM

```
sensor_msgs/msg/PointCloud2
├── header
│   ├── stamp        # Simulation timestamp
│   └── frame_id     # "lidar_link"
├── height           # 16 (vertical channels)
├── width            # 720 (horizontal samples)
├── fields[]         # x, y, z, intensity
├── is_bigendian     # false
├── point_step       # Bytes per point
├── row_step         # Bytes per row
├── data[]           # 11,520 points (720 x 16)
└── is_dense         # true (no invalid points)
```

**3D LiDAR Specifications:**
- **Horizontal**: 720 samples, 360° FOV (0.5° resolution)
- **Vertical**: 16 channels, 45° FOV (±22.5°, 2.8° resolution)
- **Total points per scan**: 11,520

**Point Fields:**
- `x`, `y`, `z`: 3D coordinates in `lidar_link` frame (meters)
- `intensity`: Return signal strength (if available)

#### LaserScan (`/lidar/scan`) - For Nav2/Visualization

```
sensor_msgs/msg/LaserScan
├── header
│   ├── stamp        # Simulation timestamp
│   └── frame_id     # "lidar_link"
├── angle_min        # -3.14159 rad (-180°)
├── angle_max        # 3.14159 rad (+180°)
├── angle_increment  # 0.00873 rad (0.5°)
├── time_increment   # Time between measurements
├── scan_time        # 0.05 s (20 Hz)
├── range_min        # 0.1 m
├── range_max        # 30.0 m
├── ranges[]         # 720 distance measurements
└── intensities[]    # 720 intensity values (if available)
```

---

## Usage

### 1. Start PX4 SITL Simulation

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=baylands_custom make px4_sitl gz_x500_lidar_360
```

### 2. Start Gazebo-ROS2 Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$(ros2 pkg prefix px4_offboard_sim)/share/px4_offboard_sim/config/gz_bridge.yaml
```

### 3. Publish Static Transforms

```bash
ros2 run tf2_ros static_transform_publisher \
  0 0 0.26 0 0 0 base_link lidar_link
```

### 4. Verify Topics

```bash
# List available topics
ros2 topic list

# Check IMU data
ros2 topic echo /imu/data

# Check LiDAR PointCloud data (for SLAM)
ros2 topic echo /lidar/points

# Check LiDAR LaserScan data (for Nav2)
ros2 topic echo /lidar/scan

# Check topic rates
ros2 topic hz /imu/data      # Should be ~200 Hz
ros2 topic hz /lidar/points  # Should be ~20 Hz
```

### 5. Visualize in RViz2

```bash
ros2 run rviz2 rviz2
```

Add displays:
- **PointCloud2**: Topic `/lidar/points`, Frame `lidar_link` (for SLAM visualization)
- **LaserScan**: Topic `/lidar/scan`, Frame `lidar_link` (for 2D visualization)
- **TF**: Show transform tree
- **Axes**: Visualize coordinate frames

---

## Troubleshooting

### No data on ROS2 topics

1. Verify Gazebo is publishing:
   ```bash
   gz topic -l | grep x500_lidar_360
   gz topic -e -t /world/baylands_custom/model/x500_lidar_360/link/lidar_link/sensor/lidar_360/scan
   ```

2. Check bridge is running:
   ```bash
   ros2 node list | grep bridge
   ```

3. Verify config file path is correct

### Frame ID mismatch

Ensure the `gz_frame_id` in the Gazebo model SDF matches the expected ROS2 frame:
- IMU: `base_link`
- LiDAR: `lidar_link`

### Clock synchronization issues

Use simulation time in ROS2:
```bash
ros2 param set /your_node use_sim_time true
```

Or in launch file:
```python
Node(
    ...
    parameters=[{'use_sim_time': True}],
)
```
