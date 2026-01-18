from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(package="nsdron_ros2", executable="pose_bridge", output="screen"),
            Node(package="nsdron_ros2", executable="mission_node", output="screen"),
            Node(package="nsdron_ros2", executable="control_node", output="screen"),
            Node(package="nsdron_ros2", executable="mavlink_bridge", output="screen"),
        ]
    )
