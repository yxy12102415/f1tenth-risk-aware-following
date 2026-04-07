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
            executable='mpc_node.py',
            name='opp_mpc_node',
            parameters=[{
                'waypoints_path': waypoint_path,
                'pose_topic': '/opp_racecar/odom',
                'drive_topic': '/opp_drive',
            }],
            output='screen',
        ),
        Node(
            package='mpc',
            executable='opponent_ekf_tracker.py',
            name='opp_ekf_tracker',
            parameters=[{
                'measurement_topic': '/ego_racecar/opp_odom',
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
                'scan_topic': '/scan',
                'follow_distance': 1.0,
                'target_timeout': 0.5,
                'min_speed_command': 0.3,
                'max_speed': 2.2,
                'max_accel': 1.0,
                'startup_delay': 1.0,
                'wall_stop_distance': 0.45,
                'wall_slow_distance': 0.8,
                'side_wall_distance': 0.6,
            }],
            output='screen',
        ),
    ])
