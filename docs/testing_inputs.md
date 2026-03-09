# Testing Keyboard & Joystick Inputs

This guide covers how to verify that keyboard and Jumper T-Pro V2 joystick data is flowing correctly through the system, using both the provided test scripts and manual ROS2 commands.

## Test Scripts

### Joystick (Jumper T-Pro V2)

```bash
# Auto-detect device
./scripts/test_joystick.sh

# Specify device manually
./scripts/test_joystick.sh /dev/input/js0
```

The script runs 5 checks:
1. Detects joystick devices at `/dev/input/js*`
2. Looks for EdgeTX USB ID (`1209:4f54`)
3. Reads device name and capabilities
4. Reads raw axis/button events for 10 seconds via `jstest`
5. Checks the ROS2 `/px4_offboard_sim/joy` topic for data

### Keyboard

```bash
# Requires sim.launch.py or joy_control_node running
./scripts/test_keyboard.sh
```

The script checks:
1. `joy_control` and `offboard_control` nodes are running
2. `cmd_vel` and `arm` topics exist and have the correct types
3. Monitors `cmd_vel` for 5 seconds (press WASD/arrows in the joy_control terminal)
4. Waits for an arm message (press SPACE in the joy_control terminal)

## Manual Testing with ROS2 Commands

### Prerequisites

Source the workspace in every terminal you use:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### 1. Check that nodes are running

```bash
ros2 node list
```

Expected output (when sim is running with `joy:=true`):

```
/joy_control
/joy_node
/offboard_control
...
```

### 2. List active topics

```bash
ros2 topic list | grep px4_offboard_sim
```

Expected:

```
/px4_offboard_sim/joy
/px4_offboard_sim/offboard_control/arm
/px4_offboard_sim/offboard_control/cmd_vel
/px4_offboard_sim/offboard_control/target_pose
/px4_offboard_sim/offboard_control/mpc_setpoint
/px4_offboard_sim/camera/image
/px4_offboard_sim/depth_camera/depth_image
/px4_offboard_sim/lidar/points
...
```

### 3. Test joystick raw data

Echo the joy topic to see raw axis and button values from the T-Pro:

```bash
ros2 topic echo /px4_offboard_sim/joy
```

Move the sticks and flip switches. You should see output like:

```yaml
header:
  stamp:
    sec: 1234
    nanosec: 567000000
  frame_id: ''
axes:
- 0.0     # Axis 0: CH1 Aileron  (Right Stick X) → Roll
- 0.0     # Axis 1: CH2 Elevator (Right Stick Y) → Pitch
- -1.0    # Axis 2: CH3 Throttle (Left Stick Y)  → Throttle
- 0.0     # Axis 3: CH4 Rudder   (Left Stick X)  → Yaw
- -1.0    # Axis 4: CH5 SA switch (3-pos)
- 0.0     # Axis 5: CH6 SB switch (3-pos, speed preset)
buttons:
- 0       # Button 0: CH9  SC (momentary, arm)
- 0       # Button 1: CH10 SD (momentary, disarm)
```

Axis values range from `-1.0` to `1.0`. If all values stay at `0.0` when you move the sticks, check:
- The radio is in **USB Joystick** mode (not USB Storage)
- EdgeTX channel order is **AETR** (Radio Setup > Default Channel Order)
- The correct device is being used (`ls /dev/input/js*`)

Check the publish rate:

```bash
ros2 topic hz /px4_offboard_sim/joy
```

Expected: ~20 Hz (set by `autorepeat_rate` in the launch file).

### 4. Test keyboard → cmd_vel

Echo the velocity commands published by the joy_control_node:

```bash
ros2 topic echo /px4_offboard_sim/offboard_control/cmd_vel
```

Then press movement keys (**in the terminal where joy_control_node is running**):

| Key | Expected effect |
|---|---|
| W | `linear.z` goes positive (throttle up) |
| S | `linear.z` goes negative (throttle down) |
| A | `angular.z` goes positive (yaw left) |
| D | `angular.z` goes negative (yaw right) |
| ↑ | `linear.x` goes positive (pitch forward) |
| ↓ | `linear.x` goes negative (pitch backward) |
| ← | `linear.y` goes positive (roll left) |
| → | `linear.y` goes negative (roll right) |

Output looks like:

```yaml
linear:
  x: 0.3    # forward/back
  y: 0.0    # left/right
  z: 0.0    # up/down
angular:
  x: 0.0
  y: 0.0
  z: 0.0    # yaw
```

If all values stay at `0.0`:
- Keyboard input only works in the **terminal where joy_control_node is running** (it reads raw stdin)
- If launched via `sim.launch.py`, that terminal captures the keyboard

Check the publish rate:

```bash
ros2 topic hz /px4_offboard_sim/offboard_control/cmd_vel
```

Expected: ~50 Hz (set by `control_rate` parameter).

### 5. Test arm/disarm

Listen for arm messages:

```bash
ros2 topic echo /px4_offboard_sim/offboard_control/arm
```

Then either press **SPACE** in the joy_control terminal, or press **SC + SD simultaneously** on the T-Pro.

You can also send arm/disarm manually:

```bash
# Arm
ros2 topic pub --once /px4_offboard_sim/offboard_control/arm std_msgs/msg/Bool "{data: true}"

# Disarm
ros2 topic pub --once /px4_offboard_sim/offboard_control/arm std_msgs/msg/Bool "{data: false}"
```

### 6. Verify joystick → cmd_vel end-to-end

With the sim running, move the T-Pro sticks while watching cmd_vel:

```bash
ros2 topic echo /px4_offboard_sim/offboard_control/cmd_vel
```

The joystick overrides keyboard input. Velocity values should change smoothly as you move the sticks (ramping is applied by joy_control_node).

### 7. Single-message capture

To grab just one message (useful for quick checks):

```bash
ros2 topic echo /px4_offboard_sim/joy --once
ros2 topic echo /px4_offboard_sim/offboard_control/cmd_vel --once
ros2 topic echo /px4_offboard_sim/offboard_control/arm --once
```

## Jumper T-Pro V2 Setup Checklist

1. **Channel order**: AETR (Radio Setup > Default Channel Order)
2. **USB mode**: Classic (Radio Setup > USB Joystick mode)
3. **Mixes page** — assign switches to channels:
   - CH5 = SA (3-pos, flight mode)
   - CH6 = SB (3-pos, speed preset)
   - CH9 = SC (momentary) → Button 0
   - CH10 = SD (momentary) → Button 1
4. Connect USB-C and select **USB Joystick** on the radio screen
5. Verify at the OS level first: `jstest /dev/input/js0`

## Troubleshooting

| Problem | Cause | Fix |
|---|---|---|
| No `/dev/input/js*` devices | Radio not in USB Joystick mode | Select "USB Joystick" on the radio screen |
| `jstest` shows data but ROS2 joy topic is empty | `joy_node` not running | Launch with `joy:=true` or run `ros2 run joy joy_node` |
| Joy topic has data but `cmd_vel` is all zeros | Axis mapping mismatch | Check `config/joy_control_tpro.yaml` axis indices |
| Keyboard keys don't register | Wrong terminal | Press keys in the terminal running `joy_control_node` |
| Joystick axes are inverted | EdgeTX channel reverse | Fix in EdgeTX Outputs page (invert the channel) |
| Only 4 axes show up (no switches) | Missing CH5-CH10 mixes | Add mixes for switch channels in EdgeTX |
| Arm combo doesn't work | SC/SD not on CH9/CH10 | Verify mixes: CH9=SC, CH10=SD (momentary) |
