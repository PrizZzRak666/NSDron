#!/usr/bin/env python3
"""Pose bridge scaffold.

Subscribes to /slam/pose and republishes to /state/local_pose, allowing
frame conversion if required.
"""

import sys


def main() -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped
    except Exception as exc:
        print("ROS2 not available. Install rclpy and run via ros2.")
        print(f"Import error: {exc}")
        return 1

    class PoseBridge(Node):
        def __init__(self) -> None:
            super().__init__("pose_bridge")
            self.sub = self.create_subscription(
                PoseStamped, "/slam/pose", self.on_pose, 10
            )
            self.pub = self.create_publisher(PoseStamped, "/state/local_pose", 10)

        def on_pose(self, msg: PoseStamped) -> None:
            # TODO: convert frames if SLAM provides ENU and control uses NED.
            self.pub.publish(msg)

    rclpy.init()
    node = PoseBridge()
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
