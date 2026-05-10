#!/usr/bin/env bash
set -e

cd /root/F1-Tenth-Duke-local/Code/sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select mpc
source install/setup.bash
ros2 launch mpc opp_mpc_ego_follow_launch.py
