#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
  CLEAN_LD_LIBRARY_PATH=""
  OLD_IFS="$IFS"
  IFS=":"
  for path in $LD_LIBRARY_PATH; do
    case "$path" in
      /snap/*) ;;
      *) CLEAN_LD_LIBRARY_PATH="${CLEAN_LD_LIBRARY_PATH:+$CLEAN_LD_LIBRARY_PATH:}$path" ;;
    esac
  done
  IFS="$OLD_IFS"
  export LD_LIBRARY_PATH="$CLEAN_LD_LIBRARY_PATH"
fi

if [ -n "${ROS_DISTRO:-}" ]; then
  ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f /opt/ros/humble/setup.bash ]; then
  ROS_SETUP="/opt/ros/humble/setup.bash"
else
  ROS_SETUP="/opt/ros/foxy/setup.bash"
fi
if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS 2 setup file not found: $ROS_SETUP" >&2
  echo "Install ROS 2 first, or set ROS_DISTRO to an installed distro such as humble." >&2
  exit 1
fi
source "$ROS_SETUP"
export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export ROS_LOG_DIR="$PWD/log/ros"
mkdir -p "$ROS_LOG_DIR"
colcon build --packages-select f1tenth_mujoco_ros mpc
source install/setup.bash
ros2 launch f1tenth_mujoco_ros mujoco_follow_launch.py "$@"
