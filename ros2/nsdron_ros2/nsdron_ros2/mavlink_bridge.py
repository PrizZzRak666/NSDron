#!/usr/bin/env python3
"""MAVLink bridge scaffold.

Publishes /control/setpoint to ArduPilot via MAVLink SET_POSITION_TARGET_LOCAL_NED.
This is a minimal template; tune frame, rate, and safety checks before flight.
"""

import sys
from typing import Optional


def main() -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
    except Exception as exc:
        print("ROS2 not available. Install rclpy and run via ros2.")
        print(f"Import error: {exc}")
        return 1

    try:
        from pymavlink import mavutil
    except Exception as exc:
        print("pymavlink not available. Install pymavlink.")
        print(f"Import error: {exc}")
        return 1

    class MavlinkBridge(Node):
        def __init__(self) -> None:
            super().__init__("mavlink_bridge")
            self.declare_parameter("mavlink_url", "udp:127.0.0.1:14550")
            self.declare_parameter("frame", 1)  # MAV_FRAME_LOCAL_NED
            self.declare_parameter("rate_hz", 20.0)
            self.mav: Optional[mavutil.mavfile] = None
            self.connect_mavlink()
            self.sub = self.create_subscription(
                TwistStamped, "/control/setpoint", self.on_setpoint, 10
            )

        def connect_mavlink(self) -> None:
            url = self.get_parameter("mavlink_url").get_parameter_value().string_value
            self.get_logger().info(f"Connecting MAVLink: {url}")
            self.mav = mavutil.mavlink_connection(url)
            self.mav.wait_heartbeat(timeout=30)
            self.get_logger().info("MAVLink heartbeat received.")

        def on_setpoint(self, msg: TwistStamped) -> None:
            if self.mav is None:
                return
            frame = self.get_parameter("frame").get_parameter_value().integer_value
            # Velocity setpoints in local NED frame. Position/yaw fields ignored.
            self.mav.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.mav.target_system,
                self.mav.target_component,
                frame,
                0b0000111111000111,  # ignore pos, accel, yaw
                0.0,
                0.0,
                0.0,
                float(msg.twist.linear.x),
                float(msg.twist.linear.y),
                float(msg.twist.linear.z),
                0.0,
                0.0,
                0.0,
                0.0,
                float(msg.twist.angular.z),
            )

    rclpy.init()
    node = MavlinkBridge()
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
