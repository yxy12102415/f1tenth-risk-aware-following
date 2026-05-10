from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mpc",
            executable="corridor_track_node.py",
            name="corridor_track_node",
            parameters=[
                {
                    "scan_topic": "/scan",
                    "drive_topic": "/drive",
                    "left_angle_deg": 55.0,
                    "right_angle_deg": -55.0,
                    "front_angle_deg": 0.0,
                    "sample_half_width_deg": 8.0,
                    "wall_balance_gain": 0.9,
                    "forward_gain": 0.35,
                    "max_steering": 0.34,
                    "min_speed": 0.35,
                    "max_speed": 0.8,
                    "front_slowdown_distance": 2.5,
                    "base_speed": 0.65,
                }
            ],
            output="screen",
        ),
    ])
