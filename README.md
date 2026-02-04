# PX4 Offboard Simulation

ROS2 package for PX4 offboard control and SLAM simulation with Gazebo Harmonic.

## Features

- **Offboard velocity control** for PX4 drones via ROS2
- **Keyboard and joystick control** (PS4 controller support)
- **Gazebo-ROS2 bridge** configuration for sensor data
- **3D LiDAR simulation** (360° horizontal, 45° vertical FOV)
- **IMU data** at 200 Hz for state estimation
- **SLAM-ready** sensor configuration with documented extrinsics

## Requirements

- ROS2 Jazzy
- Gazebo Harmonic
- PX4 Autopilot (v1.15+)
- MicroXRCE-DDS Agent

## Installation

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone <repository-url> px4_offboard_sim

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select px4_offboard_sim
source install/setup.bash
```

## Quick Start

### 1. Start PX4 SITL with Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=baylands_custom make px4_sitl gz_x500_lidar_360
```

### 2. Start MicroXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

### 3. Start Gazebo-ROS2 Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$(ros2 pkg prefix px4_offboard_sim)/share/px4_offboard_sim/config/gz_bridge.yaml
```

### 4. Run Offboard Control

```bash
ros2 run px4_offboard_sim offboard_control
ros2 run px4_offboard_sim joy_control
```

## Package Structure

```
px4_offboard_sim/
├── config/
│   └── gz_bridge.yaml          # Gazebo-ROS2 bridge configuration
├── docs/
│   ├── sensor_configuration.md # Sensor specs and extrinsics
│   └── custom_gazebo_models.md # Custom world/model guide
├── launch/
│   ├── slam_simulation.launch.py
│   └── px4_offboard_ros2_control.launch.py
├── px4_offboard_sim/
│   ├── offboard_control.py     # Main offboard control node
│   ├── joy_control.py          # Keyboard/joystick input
│   └── processes.py            # Process launcher
├── resource/
│   ├── drone_tf_params.yaml
│   └── visualize.rviz
└── scripts/
    └── run_simulation.sh       # Tmux-based full stack launcher
```

## Sensor Configuration

### Gazebo Topics (Short Names)

| Sensor | Gazebo Topic | Message Type |
|--------|--------------|--------------|
| 3D LiDAR | `/lidar/points` | `gz.msgs.PointCloudPacked` |
| LiDAR Scan | `/lidar` | `gz.msgs.LaserScan` |
| IMU | `/imu` | `gz.msgs.IMU` |
| Clock | `/clock` | `gz.msgs.Clock` |

### ROS2 Topics (Bridged)

| Topic | Message Type | Rate |
|-------|--------------|------|
| `/lidar/points` | `sensor_msgs/msg/PointCloud2` | 20 Hz |
| `/lidar/scan` | `sensor_msgs/msg/LaserScan` | 20 Hz |
| `/imu/data` | `sensor_msgs/msg/Imu` | 200 Hz |
| `/clock` | `rosgraph_msgs/msg/Clock` | Sim time |

### Sensor Extrinsics

| Sensor | Frame | Position (relative to base_link) |
|--------|-------|----------------------------------|
| IMU | `base_link` | (0, 0, 0) m |
| LiDAR | `lidar_link` | (0, 0, 0.26) m |

**Transform base_link → lidar_link:**
```
Translation: [0, 0, 0.26]
Rotation: [0, 0, 0, 1] (identity quaternion)
```

## 3D LiDAR Specifications

| Parameter | Value |
|-----------|-------|
| Horizontal FOV | 360° |
| Horizontal Samples | 720 (0.5° resolution) |
| Vertical FOV | 45° (±22.5°) |
| Vertical Channels | 16 (2.8° resolution) |
| Points per Scan | 11,520 |
| Range | 0.1 - 30.0 m |
| Update Rate | 20 Hz |

## Keyboard Controls

| Key | Action |
|-----|--------|
| W | Up (Z+) |
| S | Down (Z-) |
| A | Yaw Left |
| D | Yaw Right |
| ↑ | Pitch Forward |
| ↓ | Pitch Backward |
| ← | Roll Left |
| → | Roll Right |
| Space | Arm/Disarm |

## Custom Gazebo Models

This package is designed to work with custom PX4 Gazebo models:

- **World:** `baylands_custom` - Baylands park environment
- **Drone:** `x500_lidar_360` - X500 with 3D LiDAR
- **LiDAR:** `lidar_360` - Custom 3D scanning LiDAR

See [docs/custom_gazebo_models.md](docs/custom_gazebo_models.md) for details on modifying these models.

## PX4 Gazebo Model Files

The following files in PX4-Autopilot are used by this package:

```
PX4-Autopilot/
├── Tools/simulation/gz/
│   ├── worlds/baylands_custom.sdf
│   └── models/
│       ├── lidar_360/
│       └── x500_lidar_360/
└── ROMFS/px4fmu_common/init.d-posix/airframes/
    └── 4022_gz_x500_lidar_360
```

## Documentation

- [Sensor Configuration](docs/sensor_configuration.md) - Detailed sensor specs, extrinsics, and bridge configuration
- [Custom Gazebo Models](docs/custom_gazebo_models.md) - Guide to modifying world and model files

## Troubleshooting

### No data on ROS2 topics

1. Verify Gazebo topics exist:
   ```bash
   gz topic -l | grep -E "^/(imu|lidar)"
   ```

2. Check bridge is running:
   ```bash
   ros2 node list | grep bridge
   ```

3. Verify topic data in Gazebo:
   ```bash
   gz topic -e -t /imu
   ```

### Clock synchronization issues

Enable simulation time in your nodes:
```python
parameters=[{'use_sim_time': True}]
```

## License

MIT License

## Author

Kevin Medrano - kevin.ejem18@gmail.com
