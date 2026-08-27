import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    share_directory = get_package_share_directory("f1tenth_mujoco_ros")
    config_path = os.path.join(share_directory, "config", "mujoco.yaml")
    model_path = os.path.join(share_directory, "models", "f1tenth_scene.xml")
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
                config_path,
                {
                    "model_path": model_path,
                    "use_viewer": ParameterValue(use_viewer, value_type=bool),
                    "follow_camera": ParameterValue(follow_camera, value_type=bool),
                },
            ],
            output="screen",
        ),
    ])
