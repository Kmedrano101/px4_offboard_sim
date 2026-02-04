#!/bin/bash

# PX4 Offboard Simulation - tmux launcher script
# Author: Kevin Medrano Ayala
# This script launches all necessary nodes in a single tmux window with 5 panes

SESSION_NAME="px4_offboard_sim"
WORKSPACE_DIR="$HOME/ros2_ws"

# Check if tmux is installed
if ! command -v tmux &> /dev/null; then
    echo "Error: tmux is not installed. Please install it with: sudo apt install tmux"
    exit 1
fi

# Kill existing session if it exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Source ROS2 workspace command
SOURCE_CMD="source /opt/ros/\$ROS_DISTRO/setup.bash && source $WORKSPACE_DIR/install/setup.bash"

# Create new tmux session with first pane
tmux new-session -d -s $SESSION_NAME -n "simulation"

# === Create 5-pane layout (2 on top, 3 on bottom) ===

# Split horizontally (creates pane 1 on the right)
tmux split-window -h -t $SESSION_NAME:0.0

# Split pane 0 vertically (creates pane 2 below pane 0)
tmux split-window -v -t $SESSION_NAME:0.0

# Split pane 1 vertically (creates pane 3 below pane 1)
tmux split-window -v -t $SESSION_NAME:0.1

# Split pane 3 horizontally (creates pane 4 on the right of pane 3)
tmux split-window -h -t $SESSION_NAME:0.3

# Apply tiled layout for equal sizing
tmux select-layout -t $SESSION_NAME:0 tiled

# === Send commands to each pane ===

# Pane 0: MicroXRCE-DDS Agent
tmux send-keys -t $SESSION_NAME:0.0 "echo '=== MicroXRCE-DDS Agent ===' && MicroXRCEAgent udp4 -p 8888" C-m

# Pane 1: PX4 SITL Simulation
tmux send-keys -t $SESSION_NAME:0.1 "echo '=== PX4 SITL Simulation ===' && cd ~/PX4-Autopilot && make px4_sitl gz_x500_depth" C-m

# Pane 2: Gazebo-ROS2 Bridge
tmux send-keys -t $SESSION_NAME:0.2 "$SOURCE_CMD && echo '=== Gazebo-ROS2 Bridge ===' && sleep 10 && ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$WORKSPACE_DIR/install/px4_offboard_sim/share/px4_offboard_sim/resource/gz_ros2_bridge_list.yaml" C-m

# Pane 3: Offboard Control (includes frame remapping)
tmux send-keys -t $SESSION_NAME:0.3 "$SOURCE_CMD && echo '=== Offboard Control ===' && sleep 12 && ros2 run px4_offboard_sim offboard_control" C-m

# Pane 4: Joy Control (interactive keyboard input)
tmux send-keys -t $SESSION_NAME:0.4 "$SOURCE_CMD && echo '=== Joy Control (Use WASD + Arrows) ===' && sleep 15 && ros2 run px4_offboard_sim joy_control" C-m

# Select the joy control pane for keyboard input
tmux select-pane -t $SESSION_NAME:0.4

echo "=========================================="
echo "PX4 Offboard Simulation started in tmux"
echo "=========================================="
echo ""
echo "Layout (5 panes):"
echo "  +---------------------+---------------------+"
echo "  | 0: XRCE Agent       | 1: PX4 SITL         |"
echo "  +---------------------+---------------------+"
echo "  | 2: GZ Bridge        | 3: Offboard | 4: Joy|"
echo "  +---------------------+-------------+-------+"
echo ""
echo "Commands:"
echo "  Attach:       tmux attach -t $SESSION_NAME"
echo "  Detach:       Ctrl+b, then d"
echo "  Switch pane:  Ctrl+b, then arrow keys"
echo "  Kill:         tmux kill-session -t $SESSION_NAME"
echo "  Zoom pane:    Ctrl+b, then z (toggle)"
echo ""

# Attach to the session
tmux attach -t $SESSION_NAME
