#!/usr/bin/env bash
set -e

cd /root/F1-Tenth-Duke-local/Code/sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select mpc
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
