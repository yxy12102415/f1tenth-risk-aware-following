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
  exit 1
fi

source "$ROS_SETUP"
source install/setup.bash

export LD_LIBRARY_PATH="/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu:${LD_LIBRARY_PATH:-}"
export XDG_DATA_HOME="${XDG_DATA_HOME:-$HOME/.local/share}"
export XDG_DATA_DIRS="/usr/local/share:/usr/share"

env \
  -u SNAP -u SNAP_ARCH -u SNAP_COMMON -u SNAP_CONTEXT \
  -u SNAP_COOKIE -u SNAP_DATA -u SNAP_EUID -u SNAP_INSTANCE_NAME \
  -u SNAP_LAUNCHER_ARCH_TRIPLET -u SNAP_LIBRARY_PATH -u SNAP_NAME \
  -u SNAP_REAL_HOME -u SNAP_REVISION -u SNAP_UID -u SNAP_USER_COMMON \
  -u SNAP_USER_DATA -u SNAP_VERSION \
  -u GTK_EXE_PREFIX -u GTK_IM_MODULE_FILE -u GTK_PATH \
  -u GIO_MODULE_DIR \
  rviz2 -d "$PWD/mujoco_follow.rviz" "$@"
