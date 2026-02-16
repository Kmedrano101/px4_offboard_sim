# PX4 Offboard Simulation

ROS2 (C++/Python) package for PX4 offboard drone control with Gazebo Harmonic simulation. Provides a full-stack solution including a flight control state machine, keyboard/gamepad input, 3D LiDAR and IMU simulation, and SLAM-ready sensor bridging.

## Features

- **Offboard flight control state machine** — IDLE → STREAMING → ARMING → TAKEOFF → HOVER → LANDING with automatic mode switching between position-hold and velocity control
- **Keyboard and gamepad input** — terminal keyboard control, PS4/Xbox gamepad, and Jumper T-Pro V2 RC transmitter support with velocity ramping and exponential decay
- **3D LiDAR simulation** — 360° horizontal, 45° vertical FOV, 11,520 points/scan at 20 Hz
- **IMU at 200 Hz** for real-time state estimation
- **Gazebo-ROS2 bridge** with configurable sensor topic mapping
- **One-command launch** via `sim.launch.py` that orchestrates PX4 SITL, XRCE-DDS, Gazebo bridge, control nodes, and TF publishers
- **SLAM integration** with RTAB-Map launch file and sensor extrinsics

## Requirements

- ROS2 Jazzy
- Gazebo Harmonic
- PX4 Autopilot (v1.15+)
- MicroXRCE-DDS Agent
- `px4_msgs` ROS2 package

## Installation

```bash
# Clone into your ROS2 workspace
cd ~/ros2_ws/src
git clone <repository-url> px4_offboard_sim

# Install ROS2 dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select px4_offboard_sim
source install/setup.bash
```

## Quick Start

### Option A — Full stack with `sim.launch.py` (recommended)

This single command launches everything: PX4 SITL, MicroXRCE-DDS Agent, Gazebo-ROS2 bridge, offboard control node, joy control node, and static TF publishers.

```bash
ros2 launch px4_offboard_sim sim.launch.py
```

Launch arguments:

| Argument | Default | Description |
|----------|---------|-------------|
| `world` | `baylands_custom` | Gazebo world name (must match SDF filename) |
| `px4_autopilot_dir` | `~/PX4-Autopilot` | Path to PX4-Autopilot checkout |
| `joy` | `true` | Launch joy_node + joy_control_node for gamepad/keyboard |
| `offboard` | `true` | Launch offboard control node |
| `rviz` | `false` | Launch RViz2 with sensor visualization (LiDAR, depth camera, RGB, TF) |

```bash
ros2 launch px4_offboard_sim sim.launch.py \
  world:=baylands_custom \
  px4_autopilot_dir:=~/PX4-Autopilot \
  joy:=true \
  offboard:=true \
  rviz:=true
```

### Option B — Manual step-by-step

#### 1. Start PX4 SITL with Gazebo

```bash
cd ~/PX4-Autopilot
PX4_GZ_WORLD=baylands_custom make px4_sitl gz_x500_depth
```

#### 2. Start MicroXRCE-DDS Agent

```bash
MicroXRCEAgent udp4 -p 8888
```

#### 3. Start Gazebo-ROS2 Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=$(ros2 pkg prefix px4_offboard_sim)/share/px4_offboard_sim/config/gz_bridge.yaml
```

#### 4. Run Control Nodes

```bash
ros2 launch px4_offboard_sim offboard_control.launch.py
ros2 launch px4_offboard_sim joy_control.launch.py controller:=tpro
```

### Option C — Tmux multi-pane session

```bash
bash scripts/run_simulation.sh
# Attach: tmux attach -t px4_offboard_sim
# Kill:   tmux kill-session -t px4_offboard_sim
```

## Architecture

```
Gamepad / Keyboard
       │
       ▼
 JoyControlNode          (velocity ramping, deadzone filtering)
       │
       ▼  /px4_offboard_sim/offboard_control/cmd_vel (geometry_msgs/Twist)
       │  /px4_offboard_sim/offboard_control/arm     (std_msgs/Bool)
       │
       ▼
 OffboardControlNode     (state machine, setpoint generation)
       │
       ▼  /fmu/in/trajectory_setpoint
       │  /fmu/in/offboard_control_mode
       │  /fmu/in/vehicle_command
       │
       ▼
   PX4 SITL  ◄──►  MicroXRCE-DDS Agent  ◄──►  ROS2 DDS
       │
       ▼
 Gazebo Harmonic         (physics, sensors)
       │
       ▼  gz transport topics
       │
       ▼
 ros_gz_bridge           (Gazebo → ROS2 message translation)
       │
       ▼  /px4_offboard_sim/{lidar/points, imu/data, camera/image, ...}, /clock
```

## Package Structure

```
px4_offboard_sim/
├── src/
│   ├── offboard_control_node.cpp    # Flight control state machine (C++)
│   └── joy_control_node.cpp         # Keyboard/gamepad input handler (C++)
├── px4_offboard_sim/
│   ├── __init__.py
│   └── processes.py                 # Background process launcher (Python)
├── launch/
│   ├── sim.launch.py                # Full-stack master launcher
│   ├── offboard_control.launch.py   # Standalone offboard control
│   ├── joy_control.launch.py        # Standalone joy with controller selection
│   ├── slam_simulation.launch.py    # RTAB-Map SLAM launcher (deprecated → fast_lio_ros2/launch/sim_slam.launch.py)
│   ├── px4_offboard_3d_mapping.launch.py
│   └── px4_offboard_ros2_control.launch.py
├── config/
│   ├── sim.yaml                     # Master simulation config
│   ├── offboard_control.yaml        # Flight control parameters
│   ├── joy_control.yaml             # PS4/Xbox gamepad config
│   ├── joy_control_tpro.yaml        # Jumper T-Pro V2 RC config
│   ├── gz_bridge.yaml               # Gazebo-ROS2 bridge (LiDAR + IMU)
│   └── bridge.yaml                  # Deprecated bridge config
├── worlds/
│   ├── baylands.sdf                 # Standard Baylands world
│   └── baylands_custom.sdf          # Baylands with test walls
├── models/                          # Custom Gazebo models
├── resource/
│   ├── drone_tf_params.yaml         # TF frame publisher params
│   ├── sim_visualize.rviz            # RViz2 visualization config (rviz:=true)
│   └── gz_ros2_bridge_list.yaml
├── docs/
│   ├── sensor_configuration.md      # Sensor specs and extrinsics
│   └── custom_gazebo_models.md      # World/model customization guide
├── scripts/
│   └── run_simulation.sh            # Tmux multi-pane launcher
└── logs/
```

## Control Nodes

### OffboardControlNode (C++)

State machine that manages the full flight lifecycle:

| State | Description |
|-------|-------------|
| `IDLE` | Waiting for arm command |
| `STREAMING` | Streaming setpoints to PX4 (must reach threshold before arming) |
| `SWITCH_TO_OFFBOARD` | Requesting offboard mode from PX4 |
| `ARMING` | Arming the vehicle |
| `TAKEOFF` | Climbing to target altitude |
| `HOVER` | Position-hold or velocity control (auto-switches based on input) |
| `LANDING` | Automated descent with disarm |

Key parameters (`config/offboard_control.yaml`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `takeoff_altitude` | 3.0 m | Target takeoff height |
| `landing_descent_speed` | 0.5 m/s | Descent rate during landing |
| `min_safe_altitude` | 1.5 m | Lowest allowed hover altitude |
| `velocity_threshold` | 0.02 | Input magnitude to trigger velocity mode |
| `position_switch_delay` | 0.5 s | Delay before switching back to position-hold |
| `streaming_threshold` | 50 | Setpoints required before arming (1 s at 50 Hz) |
| `control_rate` | 50 Hz | Main loop frequency |
| `landing_timeout` | 30 s | Max auto-land duration |

### JoyControlNode (C++)

Handles keyboard and gamepad input with velocity ramping and smoothing.

**Keyboard controls:**

| Key | Action |
|-----|--------|
| W / S | Throttle up / down (Z velocity) |
| A / D | Yaw left / right |
| Arrow Up / Down | Pitch forward / backward |
| Arrow Left / Right | Roll left / right |
| Q / E | Decrease / increase speed |
| Space | Toggle arm / disarm |
| X | Emergency stop |

**Supported controllers:**

| Controller | Config file | Launch argument |
|------------|------------|-----------------|
| PS4 / Xbox gamepad | `joy_control.yaml` | `controller:=gamepad` |
| Jumper T-Pro V2 | `joy_control_tpro.yaml` | `controller:=tpro` (default) |

**Jumper T-Pro V2 axis mapping (EdgeTX, AETR):**

| Channel | Axis | Function |
|---------|------|----------|
| CH1 (Aileron) | 0 | Roll |
| CH2 (Elevator) | 1 | Pitch |
| CH3 (Throttle) | 2 | Throttle |
| CH4 (Rudder) | 3 | Yaw |
| CH5 (SA switch) | 4 | Flight mode |
| CH6 (SB switch) | 5 | Speed preset |
| CH9 (SC switch) | Button 0 | Arm |
| CH10 (SD switch) | Button 1 | Disarm |

**Speed presets** (via CH6 switch on T-Pro):

| Preset | Linear | Angular | Use case |
|--------|--------|---------|----------|
| Slow | 0.15 m/s | 0.10 rad/s | Precision / indoor |
| Normal | 0.30 m/s | 0.20 rad/s | Default |
| Fast | 0.60 m/s | 0.40 rad/s | Open area |

## Sensor Configuration

### Gazebo → ROS2 Bridge Topics

All ROS2 topics are namespaced under `/px4_offboard_sim/`.

| Sensor | Gazebo Topic | ROS2 Topic | Message Type | Rate |
|--------|-------------|------------|--------------|------|
| RGB Camera | `.../camera_link/sensor/IMX214/image` | `/px4_offboard_sim/camera/image` | `sensor_msgs/Image` | ~7 Hz |
| Camera Info | `.../camera_link/sensor/IMX214/camera_info` | `/px4_offboard_sim/camera/camera_info` | `sensor_msgs/CameraInfo` | ~7 Hz |
| Depth Camera | `/depth_camera` | `/px4_offboard_sim/depth_camera/depth_image` | `sensor_msgs/Image` | ~5 Hz |
| Depth PointCloud | `/depth_camera/points` | `/px4_offboard_sim/depth_camera/points` | `sensor_msgs/PointCloud2` | ~5 Hz |
| 3D LiDAR (point cloud) | `.../lidar_link/sensor/lidar/scan/points` | `/px4_offboard_sim/lidar/points` | `sensor_msgs/PointCloud2` | ~13 Hz |
| 3D LiDAR (scan) | `.../lidar_link/sensor/lidar/scan` | `/px4_offboard_sim/lidar/scan` | `sensor_msgs/LaserScan` | ~7 Hz |
| IMU | `.../base_link/sensor/imu_sensor/imu` | `/px4_offboard_sim/imu/data` | `sensor_msgs/Imu` | ~170 Hz |
| Clock | `/clock` | `/clock` | `rosgraph_msgs/Clock` | Sim time |

### 3D LiDAR Specifications (Livox Mid-360-like)

| Parameter | Value |
|-----------|-------|
| Horizontal FOV | 360° |
| Horizontal samples | 1800 (0.2° resolution) |
| Vertical FOV | ~59° (-25° to +34°) |
| Vertical channels | 120 (~0.49° resolution) |
| Points per scan | 216,000 |
| Range | 0.3 – 40.0 m |
| Range resolution | 0.003 m |
| Update rate | 10 Hz |
| Noise | Gaussian, stddev 0.01 |

### Sensor Extrinsics

TF tree: `map → base_link → {lidar_link, camera_link}`

| Sensor | Frame | Position (relative to base_link) | Orientation |
|--------|-------|----------------------------------|-------------|
| IMU | `base_link` | (0, 0, 0) m | Identity |
| LiDAR | `lidar_link` | (0, 0, 0.052) m | Identity |
| RGB + Depth Camera | `camera_link` | (0.12, 0.03, 0.242) m | Identity |

## Custom Gazebo Models

The package includes custom Gazebo worlds and is designed to work with custom PX4 models:

| Asset | Name | Description |
|-------|------|-------------|
| World | `baylands` | Standard Baylands park environment |
| World | `baylands_custom` | Baylands with test walls forming a 3-sided enclosure |
| Drone | `x500_depth` | X500 quadrotor with depth camera |
| Drone | `x500_lidar_360` | X500 with 3D LiDAR (custom PX4 model) |
| Sensor | `lidar_360` | 360° × 45° scanning LiDAR |

Custom PX4 model files are expected at:

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

The `sim.launch.py` launcher can automatically symlink the worlds and models from this package into the PX4 directory.

See [docs/custom_gazebo_models.md](docs/custom_gazebo_models.md) for details on modifying these models.

## Launch Files

| Launch file | Description |
|-------------|-------------|
| `sim.launch.py` | Full-stack: PX4 SITL + XRCE-DDS + bridge + control + TF |
| `offboard_control.launch.py` | Offboard control node only |
| `joy_control.launch.py` | Joy control with controller selection (`tpro` / `gamepad`) |
| `slam_simulation.launch.py` | RTAB-Map SLAM (deprecated → `fast_lio_ros2 sim_slam.launch.py`) |
| `px4_offboard_3d_mapping.launch.py` | 3D mapping pipeline |
| `px4_offboard_ros2_control.launch.py` | Legacy launcher with RViz |

## Configuration

All configuration lives under `config/`:

| File | Purpose |
|------|---------|
| `sim.yaml` | Master config — PX4 paths, drone start position, world, DDS, bridge topics, controller selection |
| `offboard_control.yaml` | Flight parameters — altitude, speeds, thresholds, PX4 topic names |
| `joy_control.yaml` | PS4/Xbox gamepad axis mapping, speed scaling, smoothing |
| `joy_control_tpro.yaml` | Jumper T-Pro V2 channel mapping, speed presets, deadzone |
| `gz_bridge.yaml` | Gazebo-ROS2 bridge topic definitions (LiDAR + IMU) |

## Troubleshooting

### No data on ROS2 topics

1. Verify Gazebo topics exist:
   ```bash
   gz topic -l | grep -E "^/(imu|lidar|clock)"
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

Ensure nodes use simulation time:
```python
parameters=[{'use_sim_time': True}]
```

### Offboard mode rejected by PX4

The node must stream at least `streaming_threshold` (default 50) setpoints before PX4 accepts the offboard mode switch. Check that the control node is publishing to `/fmu/in/offboard_control_mode` at 50 Hz.

### Joystick not detected

```bash
# List input devices
ls /dev/input/js*

# Test joystick
jstest /dev/input/js0

# Override device in launch
ros2 launch px4_offboard_sim joy_control.launch.py joy_dev:=/dev/input/js1
```

## Documentation

- [Sensor Configuration](docs/sensor_configuration.md) — Detailed sensor specs, coordinate frames, extrinsics, and bridge architecture
- [Custom Gazebo Models](docs/custom_gazebo_models.md) — Guide to modifying worlds, drone models, and sensor parameters

## License

MIT License

## Author

Kevin Medrano — kevin.ejem18@gmail.com
