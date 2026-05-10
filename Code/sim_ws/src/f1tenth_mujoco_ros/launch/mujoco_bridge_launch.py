import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = get_package_share_directory("f1tenth_mujoco_ros")
    config_path = os.path.join(share_directory, "config", "mujoco.yaml")
    model_path = os.path.join(share_directory, "models", "f1tenth_scene.xml")

    return LaunchDescription([
        Node(
            package="f1tenth_mujoco_ros",
            executable="mujoco_bridge",
            name="mujoco_bridge",
            parameters=[
                config_path,
                {"model_path": model_path},
            ],
            output="screen",
        ),
    ])
