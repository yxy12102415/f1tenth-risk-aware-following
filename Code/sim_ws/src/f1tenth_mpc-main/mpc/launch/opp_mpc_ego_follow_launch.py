import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = os.path.join(get_package_share_directory('mpc'), 'waypoints', '')
    waypoint_path = share_directory + 'Melbourne_map_mpc.csv'

    return LaunchDescription([
        Node(
            package='mpc',
            executable='opp_pid_node.py',
            name='opp_pid_node',
            parameters=[{
                'waypoints_path': waypoint_path,
                'pose_topic': '/opp_racecar/odom',
                'drive_topic': '/opp_drive',
                'lookahead_distance': 1.8,
                'max_speed': 3.0,
                'min_speed': 0.2,
                'max_accel': 0.8,
                'kp_steer': 0.9,
                'ki_steer': 0.0,
                'kd_steer': 0.25,
                'kp_speed': 1.0,
                'ki_speed': 0.0,
                'kd_speed': 0.05,
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='opponent_ekf_tracker.py',
            name='opp_ekf_tracker',
            parameters=[{
                'measurement_source': 'lidar',
                'measurement_topic': '/ego_racecar/opp_odom',
                'scan_topic': '/scan',
                'ego_odom_topic': '/ego_racecar/odom',
                'lidar_fov_deg': 190.0,
                'min_cluster_width': 0.18,
                'max_cluster_width': 0.66,
                'min_cluster_points': 9,
                'max_cluster_points': 100,
                'association_gate': 0.7,
                'output_topic': '/ego_racecar/opp_odom_ekf',
                'output_pose_topic': '/ego_racecar/opp_odom_ekf_pose',
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='ego_mpc_follower.py',
            name='ego_mpc_follower',
            parameters=[{
                'ego_odom_topic': '/ego_racecar/odom',
                'target_odom_topic': '/ego_racecar/opp_odom_ekf',
                'drive_topic': '/drive',
                'debug_drive_topic': '/ego_mpc_debug_cmd',
                'scan_topic': '/scan',
                'follow_distance': 0.5,
                'target_timeout': 1.2,
                'min_speed_command': 0.3,
                'max_speed': 3.0,
                'max_accel': 2.0,
                'startup_delay': 1.0,
                'wall_stop_distance': 0.2,
                'wall_slow_distance': 1.0,
                'cbf_front_gamma': 2.0,
                'cbf_q_accel': 1.0,
                'cbf_q_steer': 30.0,
            }],
            output='screen',
        ),
    ])
