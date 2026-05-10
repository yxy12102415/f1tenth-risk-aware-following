import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    mujoco_share = get_package_share_directory("f1tenth_mujoco_ros")
    mpc_share = get_package_share_directory("mpc")

    mujoco_config = os.path.join(mujoco_share, "config", "mujoco.yaml")
    mujoco_model = os.path.join(mujoco_share, "models", "f1tenth_scene.xml")
    waypoint_path = os.path.join(mpc_share, "waypoints", "Melbourne_map_mpc.csv")

    use_viewer = LaunchConfiguration("use_viewer")
    follow_camera = LaunchConfiguration("follow_camera")

    return LaunchDescription([
        DeclareLaunchArgument("use_viewer", default_value="true"),
        DeclareLaunchArgument("follow_camera", default_value="false"),
        Node(
            package="f1tenth_mujoco_ros",
            executable="mujoco_bridge",
            name="mujoco_bridge",
            parameters=[
                mujoco_config,
                {
                    "model_path": mujoco_model,
                    "num_agent": 2,
                    "use_viewer": ParameterValue(use_viewer, value_type=bool),
                    "follow_camera": ParameterValue(follow_camera, value_type=bool),
                },
            ],
            output="screen",
        ),
        Node(
            package="mpc",
            executable="opp_pid_node.py",
            name="opp_pid_node",
            parameters=[{
                "waypoints_path": waypoint_path,
                "pose_topic": "/opp_racecar/odom",
                "drive_topic": "/opp_drive",
                "lookahead_distance": 1.8,
                "max_speed": 2.2,
                "min_speed": 0.2,
                "max_accel": 0.8,
                "kp_steer": 0.9,
                "ki_steer": 0.0,
                "kd_steer": 0.25,
                "kp_speed": 1.0,
                "ki_speed": 0.0,
                "kd_speed": 0.05,
            }],
            output="screen",
        ),
        Node(
            package="mpc",
            executable="opponent_ekf_tracker.py",
            name="opp_ekf_tracker",
            parameters=[{
                "measurement_source": "lidar",
                "measurement_topic": "/ego_racecar/opp_odom",
                "scan_topic": "/scan",
                "ego_odom_topic": "/ego_racecar/odom",
                "lidar_fov_deg": 190.0,
                "lidar_min_range": 0.2,
                "lidar_max_range": 8.0,
                "min_cluster_width": 0.18,
                "max_cluster_width": 0.66,
                "min_cluster_points": 9,
                "max_cluster_points": 100,
                "association_gate": 0.9,
                "output_topic": "/ego_racecar/opp_odom_ekf",
                "output_pose_topic": "/ego_racecar/opp_odom_ekf_pose",
            }],
            output="screen",
        ),
        Node(
            package="mpc",
            executable="ego_mpc_follower.py",
            name="ego_mpc_follower",
            parameters=[{
                "ego_odom_topic": "/ego_racecar/odom",
                "target_odom_topic": "/ego_racecar/opp_odom_ekf",
                "drive_topic": "/drive",
                "debug_drive_topic": "/ego_mpc_debug_cmd",
                "scan_topic": "/scan",
                "follow_distance": 0.9,
                "target_timeout": 1.2,
                "min_speed_command": 0.3,
                "max_speed": 2.8,
                "max_accel": 2.5,
                "max_steer": 0.4189,
                "startup_delay": 0.2,
                "wall_stop_distance": 0.2,
                "wall_slow_distance": 1.0,
                "cbf_front_gamma": 2.0,
                "cbf_q_accel": 1.0,
                "cbf_q_steer": 30.0,
                "hybrid_astar_yaw_bins": 36,
            }],
            output="screen",
        ),
    ])
