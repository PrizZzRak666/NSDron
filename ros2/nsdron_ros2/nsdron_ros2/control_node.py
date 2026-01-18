#!/usr/bin/env python3
"""Control node scaffold.

Consumes local pose + target and publishes velocity setpoints. MAVLink
integration can be added via MAVSDK or pymavlink.
"""

import math
import sys


def main() -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PoseStamped, TwistStamped
    except Exception as exc:
        print("ROS2 not available. Install rclpy and run via ros2.")
        print(f"Import error: {exc}")
        return 1

    class ControlNode(Node):
        def __init__(self) -> None:
            super().__init__("control_node")
            self.sub_pose = self.create_subscription(
                PoseStamped, "/state/local_pose", self.on_pose, 10
            )
            self.sub_target = self.create_subscription(
                PoseStamped, "/mission/target", self.on_target, 10
            )
            self.pub = self.create_publisher(TwistStamped, "/control/setpoint", 10)
            self.pose = None
            self.target = None

        def on_pose(self, msg: PoseStamped) -> None:
            self.pose = msg
            self.publish_setpoint()

        def on_target(self, msg: PoseStamped) -> None:
            self.target = msg
            self.publish_setpoint()

        def publish_setpoint(self) -> None:
            if self.pose is None or self.target is None:
                return
            dx = self.target.pose.position.x - self.pose.pose.position.x
            dy = self.target.pose.position.y - self.pose.pose.position.y
            dz = self.target.pose.position.z - self.pose.pose.position.z
            dist = math.sqrt(dx * dx + dy * dy + dz * dz) + 1e-6
            speed = 0.5

            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "local"
            msg.twist.linear.x = speed * dx / dist
            msg.twist.linear.y = speed * dy / dist
            msg.twist.linear.z = speed * dz / dist
            self.pub.publish(msg)

    rclpy.init()
    node = ControlNode()
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
