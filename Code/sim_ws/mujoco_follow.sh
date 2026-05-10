#!/usr/bin/env bash
set -e

cd /root/F1-Tenth-Duke-local/Code/sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select f1tenth_mujoco_ros mpc
source install/setup.bash
ros2 launch f1tenth_mujoco_ros mujoco_follow_launch.py
