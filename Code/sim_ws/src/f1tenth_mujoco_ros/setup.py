from glob import glob
import os

from setuptools import setup

package_name = "f1tenth_mujoco_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
        (os.path.join("share", package_name, "models"), glob("models/*.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="F1TENTH Duke",
    maintainer_email="f1tenth@example.com",
    description="MuJoCo bridge for F1TENTH ROS 2 controllers.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_bridge = f1tenth_mujoco_ros.mujoco_bridge:main",
        ],
    },
)
