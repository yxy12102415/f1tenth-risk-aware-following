import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = os.path.join(
        get_package_share_directory('mpc'),
        'waypoints', '')
    return LaunchDescription([
        Node(
            package='mpc',
            executable='course_waypoint_mpc.py',
            name='course_waypoint_mpc',
            parameters=[
                {
                'pose_topic': '/ego_racecar/odom',
                'drive_topic': '/drive',
                'record_trajectory': False,
                }
            ],
            output='screen',
        ),
    ])
