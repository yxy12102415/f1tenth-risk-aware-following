import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    gym_launch = os.path.join(
        get_package_share_directory("f1tenth_gym_ros"),
        "launch",
        "gym_bridge_launch.py",
    )
    follower_launch = os.path.join(
        get_package_share_directory("mpc"),
        "launch",
        "opp_mpc_ego_follow_launch.py",
    )

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gym_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(follower_launch)),
    ])
