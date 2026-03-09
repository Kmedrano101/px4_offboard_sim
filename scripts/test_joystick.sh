#!/usr/bin/env bash
# test_joystick.sh — Diagnose Jumper T-Pro V2 (or any joystick) data flow
#
# Usage:
#   ./test_joystick.sh          # auto-detect device
#   ./test_joystick.sh /dev/input/js1
#
# Prerequisites:
#   sudo apt install joystick evtest
#   EdgeTX radio: USB-C connected, "USB Joystick" selected on screen

set -euo pipefail

BOLD='\033[1m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
RESET='\033[0m'

JOY_DEV="${1:-}"

# ─── Step 0: Check dependencies ──────────────────────────────
echo -e "\n${BOLD}[0/5] Checking dependencies${RESET}"
echo "────────────────────────────────"

DEPS_MISSING=0

check_dep() {
    local cmd="$1"
    local pkg="$2"
    local desc="$3"
    if command -v "$cmd" &>/dev/null; then
        echo -e "  ${GREEN}✓${RESET} $cmd — $desc"
    else
        echo -e "  ${RED}✗${RESET} $cmd — $desc"
        echo -e "    Install: ${CYAN}$pkg${RESET}"
        DEPS_MISSING=1
    fi
}

check_dep "jstest"  "sudo apt install joystick"       "Raw joystick event reader"
check_dep "lsusb"   "sudo apt install usbutils"       "USB device listing"
check_dep "udevadm" "sudo apt install udev"            "Device info lookup"

# ROS2 dependencies (checked but non-blocking for raw tests)
if command -v ros2 &>/dev/null; then
    echo -e "  ${GREEN}✓${RESET} ros2 — ROS2 CLI"
    # Check ros2 packages
    ROS_PKG_LIST=$(ros2 pkg list 2>/dev/null || true)
    for pkg in joy px4_offboard_sim px4_msgs; do
        if echo "$ROS_PKG_LIST" | grep -qw "$pkg"; then
            echo -e "  ${GREEN}✓${RESET} ros2 pkg: $pkg"
        else
            echo -e "  ${RED}✗${RESET} ros2 pkg: $pkg — not found"
            case "$pkg" in
                joy)               echo -e "    Install: ${CYAN}sudo apt install ros-jazzy-joy${RESET}" ;;
                px4_offboard_sim)  echo -e "    Build:   ${CYAN}cd ~/ros2_ws && colcon build --packages-select px4_offboard_sim${RESET}" ;;
                px4_msgs)          echo -e "    Build:   ${CYAN}cd ~/ros2_ws && colcon build --packages-select px4_msgs${RESET}" ;;
            esac
            DEPS_MISSING=1
        fi
    done
else
    echo -e "  ${RED}✗${RESET} ros2 — ROS2 CLI not found"
    echo -e "    Source: ${CYAN}source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash${RESET}"
    DEPS_MISSING=1
fi

if [ "$DEPS_MISSING" -eq 1 ]; then
    echo ""
    echo -e "${YELLOW}Some dependencies are missing (see above). Raw joystick tests may still work.${RESET}"
else
    echo ""
    echo -e "${GREEN}All dependencies present.${RESET}"
fi

# ─── Step 1: Detect joystick device ────────────────────────────
echo -e "\n${BOLD}[1/5] Detecting joystick devices${RESET}"
echo "────────────────────────────────"

if [ ! -d /dev/input ]; then
    echo -e "${RED}ERROR: /dev/input does not exist${RESET}"
    exit 1
fi

JS_DEVICES=$(ls /dev/input/js* 2>/dev/null || true)
if [ -z "$JS_DEVICES" ]; then
    echo -e "${RED}No joystick devices found at /dev/input/js*${RESET}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Connect the Jumper T-Pro V2 via USB-C"
    echo "  2. On the radio screen, select 'USB Joystick'"
    echo "  3. Check dmesg: dmesg | tail -20"
    echo "  4. Check USB: lsusb | grep -i 'edgetx\|1209:4f54\|joystick'"
    exit 1
fi

echo "Found devices:"
for dev in $JS_DEVICES; do
    name=$(cat "/sys/class/input/$(basename "$dev")/device/name" 2>/dev/null || echo "unknown")
    echo -e "  ${GREEN}$dev${RESET} — $name"
done

# Auto-select if not specified
if [ -z "$JOY_DEV" ]; then
    JOY_DEV=$(echo "$JS_DEVICES" | head -1)
    echo -e "\nUsing: ${CYAN}$JOY_DEV${RESET} (pass a different path as argument to override)"
else
    if [ ! -e "$JOY_DEV" ]; then
        echo -e "${RED}ERROR: $JOY_DEV does not exist${RESET}"
        exit 1
    fi
fi

# ─── Step 2: USB device info ───────────────────────────────────
echo -e "\n${BOLD}[2/5] USB device info${RESET}"
echo "────────────────────────────────"

TPRO_USB=$(lsusb 2>/dev/null | grep -iE 'edgetx|1209:4f54|joystick' || true)
if [ -n "$TPRO_USB" ]; then
    echo -e "${GREEN}$TPRO_USB${RESET}"
else
    echo -e "${YELLOW}No EdgeTX/T-Pro USB device found — may be a different controller${RESET}"
    lsusb 2>/dev/null | head -10
fi

# ─── Step 3: Device capabilities ──────────────────────────────
echo -e "\n${BOLD}[3/5] Joystick capabilities ($JOY_DEV)${RESET}"
echo "────────────────────────────────"

DEV_NAME=$(cat "/sys/class/input/$(basename "$JOY_DEV")/device/name" 2>/dev/null || echo "unknown")
echo "Device name: $DEV_NAME"

if command -v udevadm &>/dev/null; then
    SYSFS="/sys/class/input/$(basename "$JOY_DEV")/device"
    NUM_AXES=$(cat "$SYSFS/id/product" 2>/dev/null || echo "?")
    echo "Product ID: $NUM_AXES"
fi

# Quick jstest check
if command -v jstest &>/dev/null; then
    echo ""
    echo "Axis/button count (from jstest):"
    # jstest --info is not available on all versions, use timeout on normal mode
    timeout 0.5 jstest "$JOY_DEV" 2>/dev/null | head -3 || true
else
    echo -e "${YELLOW}jstest not installed — install with: sudo apt install joystick${RESET}"
fi

# ─── Step 4: Raw data test (10 seconds) ───────────────────────
echo -e "\n${BOLD}[4/5] Raw joystick data test${RESET}"
echo "────────────────────────────────"
echo "Reading raw events for 10 seconds — move sticks and press buttons..."
echo ""

if command -v jstest &>/dev/null; then
    echo -e "${CYAN}Expected T-Pro axes (EdgeTX Classic, AETR):${RESET}"
    echo "  Axis 0 = CH1 Aileron  (Right Stick X) → Roll"
    echo "  Axis 1 = CH2 Elevator (Right Stick Y) → Pitch"
    echo "  Axis 2 = CH3 Throttle (Left Stick Y)  → Throttle"
    echo "  Axis 3 = CH4 Rudder   (Left Stick X)  → Yaw"
    echo "  Axis 4 = CH5 SA switch (3-pos)"
    echo "  Axis 5 = CH6 SB switch (3-pos, speed preset)"
    echo ""
    echo -e "${CYAN}Expected buttons:${RESET}"
    echo "  Button 0 = CH9  SC (momentary)"
    echo "  Button 1 = CH10 SD (momentary)"
    echo ""

    timeout 10 jstest --event "$JOY_DEV" 2>/dev/null || true
    echo ""

    EVENTS_SEEN=$?
    if [ $EVENTS_SEEN -eq 124 ]; then
        # timeout exit code = received data until timeout
        echo -e "${GREEN}✓ Raw joystick data is flowing${RESET}"
    else
        echo -e "${YELLOW}Test ended — check if you moved the sticks${RESET}"
    fi
else
    # Fallback: read raw with cat
    echo "(jstest not available, using raw read)"
    timeout 10 cat "$JOY_DEV" | od -A x -t x1 -w16 | head -20 || true
fi

# ─── Step 5: ROS2 joy topic test ──────────────────────────────
echo -e "\n${BOLD}[5/5] ROS2 joy_node topic test${RESET}"
echo "────────────────────────────────"

# Check if ROS2 is sourced
if ! command -v ros2 &>/dev/null; then
    echo -e "${YELLOW}ros2 not found in PATH — source ROS2 first:${RESET}"
    echo "  source /opt/ros/jazzy/setup.bash"
    echo "  source ~/ros2_ws/install/setup.bash"
    echo ""
    echo "Then run joy_node manually:"
    echo "  ros2 run joy joy_node --ros-args -p dev:=$JOY_DEV -p deadzone:=0.03 -r /joy:=/px4_offboard_sim/joy"
    exit 0
fi

JOY_TOPIC="/px4_offboard_sim/joy"

# Check if joy topic is already being published
TOPIC_LIST=$(ros2 topic list 2>/dev/null || true)
if echo "$TOPIC_LIST" | grep -q "$JOY_TOPIC"; then
    echo -e "${GREEN}✓ Topic $JOY_TOPIC exists${RESET}"
    echo ""
    echo "Listening for 5 seconds (move sticks)..."
    echo ""
    timeout 5 ros2 topic echo "$JOY_TOPIC" --once 2>/dev/null || true
    echo ""
    echo "Publishing rate:"
    timeout 3 ros2 topic hz "$JOY_TOPIC" 2>/dev/null || true
else
    echo -e "${YELLOW}Topic $JOY_TOPIC not found — joy_node is not running${RESET}"
    echo ""
    echo "To start joy_node standalone:"
    echo "  ros2 run joy joy_node --ros-args \\"
    echo "    -p dev:=$JOY_DEV \\"
    echo "    -p deadzone:=0.03 \\"
    echo "    -p autorepeat_rate:=20.0 \\"
    echo "    -r /joy:=/px4_offboard_sim/joy"
    echo ""
    echo "Or launch the full sim:"
    echo "  ros2 launch px4_offboard_sim sim.launch.py joy:=true"
    echo ""

    read -rp "Start a temporary joy_node now? [y/N] " answer
    if [[ "$answer" =~ ^[Yy]$ ]]; then
        echo -e "\nStarting joy_node on $JOY_DEV (Ctrl+C to stop)..."
        echo "In another terminal, check with:"
        echo "  ros2 topic echo /px4_offboard_sim/joy"
        echo ""
        ros2 run joy joy_node --ros-args \
            -p dev:="$JOY_DEV" \
            -p deadzone:=0.03 \
            -p autorepeat_rate:=20.0 \
            -r /joy:=/px4_offboard_sim/joy
    fi
fi
