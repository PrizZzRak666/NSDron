from setuptools import setup

package_name = "nsdron_ros2"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/nsdron_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="NSDron",
    maintainer_email="dev@example.com",
    description="ROS2 nodes for NSDron MVP.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "pose_bridge = nsdron_ros2.pose_bridge:main",
            "mission_node = nsdron_ros2.mission_node:main",
            "control_node = nsdron_ros2.control_node:main",
            "mavlink_bridge = nsdron_ros2.mavlink_bridge:main",
        ],
    },
)
