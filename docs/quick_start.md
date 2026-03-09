# Quick Start Guide

## Prerequisites

- **ROS2 Jazzy** — sourced via `/opt/ros/jazzy/setup.bash`
- **PX4-Autopilot** — built at `~/PX4-Autopilot` with `make px4_sitl_default`
- **Micro-XRCE-DDS-Agent** — built at `~/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent`
- **px4_msgs** — matching your PX4 version, in the same ROS2 workspace
- **Gazebo Harmonic** — installed (ships with ROS2 Jazzy)
- **ros_gz_bridge** — `sudo apt install ros-jazzy-ros-gz-bridge`
- **joy** (optional) — `sudo apt install ros-jazzy-joy` (for gamepad support)

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --packages-select px4_offboard_sim
source install/setup.bash
```

## Launch

### Basic simulation (keyboard + gamepad control)

```bash
ros2 launch px4_offboard_sim sim.launch.py
```

This starts the full stack:
1. **MicroXRCE-DDS Agent** — PX4 ↔ ROS2 bridge
2. **PX4 SITL** — autopilot with Gazebo physics
3. **Gazebo sensor bridge** — camera, depth, LiDAR, IMU topics into ROS2
4. **Offboard control node** — flight state machine
5. **Joy control node** — keyboard + gamepad velocity input
6. **Static TFs** — map → base_link → sensor frames

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `world` | `baylands_custom` | Gazebo world: `baylands`, `baylands_custom`, `obstacle_course` |
| `joy` | `true` | Enable keyboard/gamepad control |
| `offboard` | `true` | Enable offboard control node |
| `rviz` | `false` | Launch RViz2 with sensor visualization |
| `px4_autopilot_dir` | `~/PX4-Autopilot` | Path to PX4-Autopilot source |

Examples:

```bash
# Obstacle course world, no joystick
ros2 launch px4_offboard_sim sim.launch.py world:=obstacle_course joy:=false

# With RViz
ros2 launch px4_offboard_sim sim.launch.py rviz:=true
```

## Flying the Drone

### 1. Arm and takeoff

In a separate terminal:

```bash
ros2 topic pub --once /px4_offboard_sim/offboard_control/arm std_msgs/msg/Bool "{data: true}"
```

The offboard control node will automatically:
- Stream setpoints to PX4
- Switch to offboard mode
- Arm the drone
- Take off to 3.0m (configurable in `config/offboard_control.yaml`)

### 2. Control with keyboard

If `joy:=true` (default), the joy control terminal accepts keyboard input (Mode 2 RC layout):

| Key | Action |
|---|---|
| W / S | Throttle up / down |
| A / D | Yaw left / right |
| ↑ / ↓ | Pitch forward / backward |
| ← / → | Roll left / right |
| Q / E | Increase / decrease speed |
| X | Emergency stop (zero velocity) |
| Space | Toggle arm / disarm |

### 3. Land

```bash
ros2 topic pub --once /px4_offboard_sim/offboard_control/arm std_msgs/msg/Bool "{data: false}"
```

The drone will switch to AUTO_LAND mode and disarm after touchdown.

## Integration with Path Planners

The offboard control node accepts external trajectory commands in HOVER state:

### SUPER planner (via px4_super_bridge)

Publishes `geometry_msgs/PoseStamped` to `/px4_offboard_sim/offboard_control/target_pose` in ENU frame. The node handles ENU→NED conversion and frame offset calibration automatically.

```bash
# Launch SUPER planner stack
ros2 launch super_planner full_integration_test.launch.py launch_bridge:=true
```

### MPC controller

Publishes `px4_msgs/TrajectorySetpoint` (pos+vel+acc, NED frame) to `/px4_offboard_sim/offboard_control/mpc_setpoint`. MPC takes priority over the bridge.

```bash
# Launch with MPC
ros2 launch super_planner full_integration_test.launch.py use_mpc:=true
```

## ROS2 Topics

### Subscribed by offboard_control_node

| Topic | Type | Description |
|---|---|---|
| `/fmu/out/vehicle_status_v1` | `px4_msgs/VehicleStatus` | PX4 nav/arm state |
| `/fmu/out/vehicle_attitude` | `px4_msgs/VehicleAttitude` | Drone orientation |
| `/fmu/out/vehicle_local_position_v1` | `px4_msgs/VehicleLocalPosition` | Position in NED |
| `/px4_offboard_sim/offboard_control/cmd_vel` | `geometry_msgs/Twist` | Velocity commands |
| `/px4_offboard_sim/offboard_control/arm` | `std_msgs/Bool` | Arm/disarm command |
| `/px4_offboard_sim/offboard_control/target_pose` | `geometry_msgs/PoseStamped` | Planner setpoints (ENU) |
| `/px4_offboard_sim/offboard_control/mpc_setpoint` | `px4_msgs/TrajectorySetpoint` | MPC setpoints (NED) |

### Sensor topics (from Gazebo bridge)

| Topic | Type |
|---|---|
| `/px4_offboard_sim/camera/image` | `sensor_msgs/Image` |
| `/px4_offboard_sim/camera/camera_info` | `sensor_msgs/CameraInfo` |
| `/px4_offboard_sim/depth_camera/depth_image` | `sensor_msgs/Image` |
| `/px4_offboard_sim/depth_camera/points` | `sensor_msgs/PointCloud2` |
| `/px4_offboard_sim/lidar/points` | `sensor_msgs/PointCloud2` |
| `/px4_offboard_sim/lidar/scan` | `sensor_msgs/LaserScan` |
| `/px4_offboard_sim/imu/data` | `sensor_msgs/Imu` |

## Configuration

All simulation parameters live in **`config/sim.yaml`** — the single source of truth. Key sections:

- `px4` — autopilot path, sim model (`gz_x500_depth`)
- `drone_start` — spawn position/orientation
- `bridge.sensors` — Gazebo↔ROS2 topic mappings
- `frames` — TF frame names and sensor offsets
- `joy` — controller preset (`tpro` or `gamepad`)

Flight parameters are in **`config/offboard_control.yaml`** (takeoff altitude, speeds, thresholds).
