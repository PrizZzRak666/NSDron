#!/usr/bin/env python3
"""Mission node scaffold.

Publishes a sequence of 4 waypoints in local coordinates and a return-to-home
target once all waypoints are reached.
"""

import sys
from dataclasses import dataclass
from typing import List


def main() -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
    except Exception as exc:
        print("ROS2 not available. Install rclpy and run via ros2.")
        print(f"Import error: {exc}")
        return 1

    @dataclass
    class Waypoint:
        x: float
        y: float
        z: float
        yaw: float = 0.0

    class MissionNode(Node):
        def __init__(self) -> None:
            super().__init__("mission_node")
            self.pub = self.create_publisher(PoseStamped, "/mission/target", 10)
            self.timer = self.create_timer(0.5, self.on_timer)
            self.waypoints: List[Waypoint] = [
                Waypoint(2.0, 0.0, -2.0),
                Waypoint(2.0, 2.0, -2.0),
                Waypoint(0.0, 2.0, -2.0),
                Waypoint(0.0, 0.0, -2.0),
            ]
            self.index = 0

        def on_timer(self) -> None:
            if self.index >= len(self.waypoints):
                return
            wp = self.waypoints[self.index]
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "local"
            msg.pose.position.x = wp.x
            msg.pose.position.y = wp.y
            msg.pose.position.z = wp.z
            # TODO: set yaw in orientation
            self.pub.publish(msg)

    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
