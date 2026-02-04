# Custom Gazebo World and Models

This document describes the custom Gazebo world and drone model configuration for PX4 SITL simulation with 3D LiDAR for SLAM applications.

## Table of Contents

- [Overview](#overview)
- [Directory Structure](#directory-structure)
- [Custom World: baylands_custom](#custom-world-baylands_custom)
- [Custom Models](#custom-models)
- [File Reference](#file-reference)
- [Modifying the Configuration](#modifying-the-configuration)
- [Adding Custom Objects](#adding-custom-objects)

---

## Overview

The simulation environment consists of:

| Component | Description |
|-----------|-------------|
| **World** | `baylands_custom` - Based on Baylands park environment |
| **Drone** | `x500_lidar_360` - X500 quadcopter with 3D LiDAR |
| **LiDAR** | `lidar_360` - 360° horizontal, 45° vertical FOV |

---

## Directory Structure

```
PX4-Autopilot/
├── Tools/simulation/gz/
│   ├── worlds/
│   │   └── baylands_custom.sdf        # Custom world file
│   └── models/
│       ├── lidar_360/                  # 3D LiDAR sensor model
│       │   ├── model.config
│       │   └── model.sdf
│       ├── x500_lidar_360/             # Drone with 3D LiDAR + IMU
│       │   ├── model.config
│       │   └── model.sdf
│       ├── x500_base/                  # Base X500 drone (shared)
│       │   └── model.sdf
│       └── custom_objects/             # Directory for custom obstacles
│
├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   ├── 4022_gz_x500_lidar_360          # Airframe definition
│   └── CMakeLists.txt                  # Airframe registration
│
└── src/modules/simulation/gz_bridge/
    └── CMakeLists.txt                  # World registration (auto-detected)
```

---

## Custom World: baylands_custom

**File:** `Tools/simulation/gz/worlds/baylands_custom.sdf`

The world is based on the Baylands environment with:
- Realistic terrain from OpenRobotics Fuel
- Coast water model
- GPS coordinates: 37.412°N, -121.999°W
- ENU (East-North-Up) orientation

### World Configuration

```xml
<sdf version='1.9'>
  <world name='baylands_custom'>
    <!-- Physics settings -->
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- GPS origin -->
    <spherical_coordinates>
      <latitude_deg>37.412173071650805</latitude_deg>
      <longitude_deg>-121.998878727967</longitude_deg>
      <elevation>38</elevation>
    </spherical_coordinates>
  </world>
</sdf>
```

---

## Custom Models

### 1. lidar_360 (3D LiDAR Sensor)

**Directory:** `Tools/simulation/gz/models/lidar_360/`

A 3D scanning LiDAR with full 360° horizontal coverage and 45° vertical FOV.

#### Specifications

| Parameter | Value |
|-----------|-------|
| Horizontal FOV | 360° (-π to +π) |
| Horizontal Samples | 720 (0.5° resolution) |
| Vertical FOV | 45° (±22.5°) |
| Vertical Channels | 16 (2.8° resolution) |
| Points per Scan | 11,520 |
| Range | 0.1 - 30.0 m |
| Update Rate | 20 Hz |
| Topic Name | `/lidar` (short name) |

#### Key Configuration (model.sdf)

```xml
<sensor name="lidar_360" type="gpu_lidar">
  <topic>lidar</topic>
  <gz_frame_id>lidar_link</gz_frame_id>
  <update_rate>20</update_rate>
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>
        <min_angle>-0.3927</min_angle>
        <max_angle>0.3927</max_angle>
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>30</max>
    </range>
  </ray>
</sensor>
```

### 2. x500_lidar_360 (Drone Model)

**Directory:** `Tools/simulation/gz/models/x500_lidar_360/`

X500 quadcopter with integrated 3D LiDAR and dedicated IMU for SLAM.

#### Components

| Component | Position (relative to base_link) | Topic |
|-----------|----------------------------------|-------|
| Drone Base | Origin | - |
| 3D LiDAR | (0, 0, 0.26) m | `/lidar`, `/lidar/points` |
| IMU (SLAM) | (0, 0, 0) m | `/imu` |
| IMU (PX4) | (0, 0, 0) m | Internal |

#### Key Configuration (model.sdf)

```xml
<model name='x500_lidar_360'>
  <!-- Base drone -->
  <include merge='true'>
    <uri>x500</uri>
  </include>

  <!-- 3D LiDAR -->
  <include merge='true'>
    <uri>model://lidar_360</uri>
    <pose>0 0 .26 0 0 0</pose>
  </include>

  <!-- Dedicated IMU for ROS2/SLAM -->
  <link name="imu_link">
    <sensor name="imu_sensor" type="imu">
      <topic>imu</topic>
      <update_rate>200</update_rate>
    </sensor>
  </link>
</model>
```

### 3. Airframe Definition

**File:** `ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_lidar_360`

```bash
#!/bin/sh
# @name Gazebo x500 lidar 360
# @type Quadrotor

PX4_SIM_MODEL=${PX4_SIM_MODEL:=x500_lidar_360}
. ${R}etc/init.d-posix/airframes/4001_gz_x500
```

---

## File Reference

### Files to Modify for Sensor Configuration

| What to Change | File |
|----------------|------|
| LiDAR specs (FOV, samples, range) | `models/lidar_360/model.sdf` |
| LiDAR position on drone | `models/x500_lidar_360/model.sdf` |
| IMU rate | `models/x500_lidar_360/model.sdf` |
| World environment | `worlds/baylands_custom.sdf` |
| ROS2 bridge topics | `px4_offboard_sim/config/gz_bridge.yaml` |

### Files to Modify for New Models

| Action | Files to Create/Modify |
|--------|------------------------|
| New sensor model | `models/<name>/model.config`, `models/<name>/model.sdf` |
| New drone variant | `models/<name>/model.config`, `models/<name>/model.sdf` |
| New airframe | `airframes/<id>_gz_<name>`, `airframes/CMakeLists.txt` |
| New world | `worlds/<name>.sdf` |

---

## Modifying the Configuration

### Change LiDAR Resolution

Edit `Tools/simulation/gz/models/lidar_360/model.sdf`:

```xml
<horizontal>
  <samples>1440</samples>  <!-- Double horizontal resolution -->
</horizontal>
<vertical>
  <samples>32</samples>    <!-- Double vertical channels -->
</vertical>
```

### Change LiDAR Position

Edit `Tools/simulation/gz/models/x500_lidar_360/model.sdf`:

```xml
<include merge='true'>
  <uri>model://lidar_360</uri>
  <pose>0.1 0 0.3 0 0 0</pose>  <!-- X Y Z Roll Pitch Yaw -->
</include>
```

### Change IMU Rate

Edit `Tools/simulation/gz/models/x500_lidar_360/model.sdf`:

```xml
<sensor name="imu_sensor" type="imu">
  <update_rate>400</update_rate>  <!-- Increase to 400 Hz -->
</sensor>
```

### Change Sensor Topic Names

Edit the sensor's `<topic>` tag in the respective model.sdf:

```xml
<sensor name="lidar_360" type="gpu_lidar">
  <topic>my_custom_lidar</topic>  <!-- Custom topic name -->
</sensor>
```

Then update `config/gz_bridge.yaml` to match.

---

## Adding Custom Objects

### 1. Create Object Model

Create directory: `Tools/simulation/gz/models/custom_objects/my_obstacle/`

**model.config:**
```xml
<?xml version="1.0"?>
<model>
  <name>my_obstacle</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Custom obstacle for SLAM testing</description>
</model>
```

**model.sdf:**
```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="my_obstacle">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box><size>1 1 2</size></box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box><size>1 1 2</size></box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

### 2. Add to World

Edit `Tools/simulation/gz/worlds/baylands_custom.sdf`:

```xml
<include>
  <uri>model://custom_objects/my_obstacle</uri>
  <name>obstacle_1</name>
  <pose>5 0 1 0 0 0</pose>  <!-- 5m in front of spawn point -->
</include>
```

### 3. Multiple Instances

```xml
<include>
  <uri>model://custom_objects/my_obstacle</uri>
  <name>obstacle_1</name>
  <pose>5 0 1 0 0 0</pose>
</include>
<include>
  <uri>model://custom_objects/my_obstacle</uri>
  <name>obstacle_2</name>
  <pose>10 5 1 0 0 0.785</pose>  <!-- Rotated 45° -->
</include>
```

---

## Running the Simulation

```bash
# Start PX4 SITL with custom world and model
cd ~/PX4-Autopilot
PX4_GZ_WORLD=baylands_custom make px4_sitl gz_x500_lidar_360

# Verify Gazebo topics
gz topic -l | grep -E "^/(imu|lidar)"

# Start ROS2 bridge
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=/path/to/gz_bridge.yaml
```
