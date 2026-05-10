# F1TENTH MuJoCo ROS Bridge

This package provides a first MuJoCo entry point for the existing F1TENTH ROS 2 controllers.

The bridge subscribes to:

- `/drive` (`ackermann_msgs/AckermannDriveStamped`)

It publishes:

- `/ego_racecar/odom` (`nav_msgs/Odometry`)
- `map -> ego_racecar/base_link` TF

That matches the current MPC launch file, so the controller can run unchanged against the MuJoCo-backed vehicle state.

## Install Runtime Dependency

```bash
pip install mujoco
```

## Build

```bash
cd /root/F1-Tenth-Duke-local/Code/sim_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select f1tenth_mujoco_ros mpc
source install/setup.bash
```

## Run

Terminal 1:

```bash
ros2 launch f1tenth_mujoco_ros mujoco_bridge_launch.py
```

Terminal 2:

```bash
ros2 launch mpc sim_mpc_launch.py
```

Or start the bridge with:

```bash
/root/F1-Tenth-Duke-local/Code/sim_ws/mujoco.sh
```

## Run Opponent + Ego MPC Follow

This starts the MuJoCo two-car bridge, the opponent waypoint PID controller, the lidar opponent EKF, and the ego MPC follower in one launch:

```bash
/root/F1-Tenth-Duke-local/Code/sim_ws/mujoco_follow.sh
```

The red car is the front/opponent car on `/opp_drive` and `/opp_racecar/odom`.
The blue car is the ego car on `/drive` and `/ego_racecar/odom`.
The bridge publishes `/scan` from the ego car. The EKF estimates the front car on `/ego_racecar/opp_odom_ekf`, and the ego MPC follower uses that target plus the scan-based Hybrid A* path planner and spline smoother before solving MPC.

The current bridge uses a kinematic bicycle update for each F1TENTH pose and mirrors those poses into MuJoCo freejoint bodies. The Melbourne map is shown as a ground texture, and its CSV track widths are used to simulate lidar wall returns.
