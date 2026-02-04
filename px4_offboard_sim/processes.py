#!/usr/bin/env python3

__author__ = "Kevin Medrano Ayala"
__contact__ = "kevin.ejem18@gmail.com"

import subprocess
import time


def main():
    # List of commands to run
    commands = [
        # Run the Micro XRCE-DDS Agent
        "MicroXRCEAgent udp4 -p 8888",

        # Run the PX4 SITL simulation (Gazebo Harmonic for ROS2 Jazzy)
        "cd ~/PX4-Autopilot && PX4_GZ_WORLD=baylands_custom make px4_sitl gz_x500_lidar_360"

        # Run QGroundControl
        # "cd ~/qgroundcontrol && ./QGroundControl.AppImage"
    ]

    # Loop through each command in the list
    for command in commands:
        # Each command is run in a new tab of the gnome-terminal
        subprocess.run(["gnome-terminal", "--tab", "--", "bash", "-c", command + "; exec bash"])

        # Pause between each command
        time.sleep(1)


if __name__ == '__main__':
    main()