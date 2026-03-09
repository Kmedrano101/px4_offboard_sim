#!/usr/bin/env bash
# test_keyboard.sh — Test keyboard input → cmd_vel + arm data flow
#
# Verifies that joy_control_node is receiving keyboard input and
# publishing velocity commands and arm/disarm messages.
#
# Usage:
#   ./test_keyboard.sh
#
# Prerequisites:
#   - sim.launch.py running (or at least joy_control_node)
#   - source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash

set -euo pipefail

BOLD='\033[1m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
CYAN='\033[0;36m'
RESET='\033[0m'

CMD_VEL_TOPIC="/px4_offboard_sim/offboard_control/cmd_vel"
ARM_TOPIC="/px4_offboard_sim/offboard_control/arm"

echo -e "\n${BOLD}Keyboard → ROS2 Data Flow Test${RESET}"
echo "══════════════════════════════════════"

# ─── Step 0: Check dependencies ──────────────────────────────
echo -e "\n${BOLD}[0/4] Checking dependencies${RESET}"
echo "────────────────────────────────"

DEPS_MISSING=0

# ROS2 CLI
if command -v ros2 &>/dev/null; then
    echo -e "  ${GREEN}✓${RESET} ros2 — ROS2 CLI"
else
    echo -e "  ${RED}✗${RESET} ros2 — ROS2 CLI not found"
    echo -e "    Source: ${CYAN}source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash${RESET}"
    DEPS_MISSING=1
fi

# ROS2 packages
if command -v ros2 &>/dev/null; then
    ROS_PKG_LIST=$(ros2 pkg list 2>/dev/null || true)
    for pkg in px4_offboard_sim px4_msgs; do
        if echo "$ROS_PKG_LIST" | grep -qw "$pkg"; then
            echo -e "  ${GREEN}✓${RESET} ros2 pkg: $pkg"
        else
            echo -e "  ${RED}✗${RESET} ros2 pkg: $pkg — not found"
            case "$pkg" in
                px4_offboard_sim)  echo -e "    Build: ${CYAN}cd ~/ros2_ws && colcon build --packages-select px4_offboard_sim${RESET}" ;;
                px4_msgs)          echo -e "    Build: ${CYAN}cd ~/ros2_ws && colcon build --packages-select px4_msgs${RESET}" ;;
            esac
            DEPS_MISSING=1
        fi
    done
fi

if [ "$DEPS_MISSING" -eq 1 ]; then
    echo ""
    echo -e "${RED}Missing dependencies — cannot continue.${RESET}"
    exit 1
else
    echo ""
    echo -e "${GREEN}All dependencies present.${RESET}"
fi

# ─── Step 1: Check if required nodes are running ──────────────
echo -e "\n${BOLD}[1/4] Checking running nodes${RESET}"
echo "────────────────────────────────"

NODE_LIST=$(ros2 node list 2>/dev/null || true)

check_node() {
    local name="$1"
    if echo "$NODE_LIST" | grep -q "$name"; then
        echo -e "  ${GREEN}✓${RESET} $name"
        return 0
    else
        echo -e "  ${RED}✗${RESET} $name — not running"
        return 1
    fi
}

MISSING=0
check_node "/joy_control" || MISSING=1
check_node "/offboard_control" || MISSING=1

if [ "$MISSING" -eq 1 ]; then
    echo ""
    echo -e "${YELLOW}Some nodes are missing. Start the sim first:${RESET}"
    echo "  ros2 launch px4_offboard_sim sim.launch.py"
    echo ""
    echo "Or run joy_control_node standalone:"
    echo "  ros2 run px4_offboard_sim joy_control_node"
    echo ""
    read -rp "Continue anyway? [y/N] " answer
    [[ ! "$answer" =~ ^[Yy]$ ]] && exit 0
fi

# ─── Step 2: Check topics exist ───────────────────────────────
echo -e "\n${BOLD}[2/4] Checking topics${RESET}"
echo "────────────────────────────────"

TOPIC_LIST=$(ros2 topic list 2>/dev/null || true)

check_topic() {
    local topic="$1"
    local type
    if echo "$TOPIC_LIST" | grep -q "$topic"; then
        type=$(ros2 topic type "$topic" 2>/dev/null || echo "?")
        echo -e "  ${GREEN}✓${RESET} $topic [$type]"
        return 0
    else
        echo -e "  ${RED}✗${RESET} $topic — not found"
        return 1
    fi
}

check_topic "$CMD_VEL_TOPIC" || true
check_topic "$ARM_TOPIC" || true

# ─── Step 3: Monitor cmd_vel ──────────────────────────────────
echo -e "\n${BOLD}[3/4] Monitoring cmd_vel (5 seconds)${RESET}"
echo "────────────────────────────────"
echo -e "Press ${CYAN}W/S/A/D/arrows${RESET} in the joy_control terminal..."
echo ""

# Check publishing rate
HZ_OUTPUT=$(timeout 3 ros2 topic hz "$CMD_VEL_TOPIC" 2>/dev/null | head -3 || true)
if [ -n "$HZ_OUTPUT" ]; then
    echo "$HZ_OUTPUT"
    echo ""
fi

# Get one message
echo "Latest cmd_vel message:"
MSG=$(timeout 5 ros2 topic echo "$CMD_VEL_TOPIC" --once 2>/dev/null || true)
if [ -n "$MSG" ]; then
    echo -e "${GREEN}$MSG${RESET}"

    # Check if all zeros
    if echo "$MSG" | grep -qE 'x: [^0]|y: [^0]|z: [^0]'; then
        echo -e "${GREEN}✓ Non-zero velocity detected — keyboard input is working${RESET}"
    else
        echo -e "${YELLOW}All zeros — press a movement key in the joy_control terminal${RESET}"
    fi
else
    echo -e "${RED}No message received — joy_control_node may not be publishing${RESET}"
fi

# ─── Step 4: Monitor arm topic ────────────────────────────────
echo -e "\n${BOLD}[4/4] Arm topic test${RESET}"
echo "────────────────────────────────"
echo -e "Press ${CYAN}SPACE${RESET} in the joy_control terminal (or use the command below)..."
echo ""
echo "To arm manually:"
echo "  ros2 topic pub --once $ARM_TOPIC std_msgs/msg/Bool \"{data: true}\""
echo ""
echo "Waiting 10 seconds for an arm message..."

ARM_MSG=$(timeout 10 ros2 topic echo "$ARM_TOPIC" --once 2>/dev/null || true)
if [ -n "$ARM_MSG" ]; then
    echo -e "${GREEN}$ARM_MSG${RESET}"
    echo -e "${GREEN}✓ Arm message received${RESET}"
else
    echo -e "${YELLOW}No arm message received (this is normal if you didn't press SPACE)${RESET}"
fi

echo -e "\n${BOLD}Done.${RESET}"
echo "────────────────────────────────"
echo "If everything shows ✓, keyboard data is flowing correctly."
echo "If cmd_vel is all zeros, make sure you're pressing keys in the"
echo "terminal where joy_control_node is running (it needs raw stdin)."
