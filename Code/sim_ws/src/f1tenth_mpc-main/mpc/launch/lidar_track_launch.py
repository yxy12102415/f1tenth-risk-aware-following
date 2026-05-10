from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="mpc",
            executable="lidar_track_node.py",
            name="lidar_track_node",
            parameters=[
                {
                    "scan_topic": "/scan",
                    "drive_topic": "/drive",
                    "field_of_view_deg": 100.0,
                    "bubble_radius": 50,
                    "best_point_window": 10,
                    "steering_gain": 2.0,
                    "max_steering": 0.4,
                    "min_speed": 0.4,
                    "max_speed": 0.8,
                    "straights_speed": 0.9,
                    "front_slowdown_distance": 4.0,
                }
            ],
            output="screen",
        ),
    ])
