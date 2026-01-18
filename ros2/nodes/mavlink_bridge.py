#!/usr/bin/env python3
"""MAVLink bridge scaffold.

Publishes /control/setpoint to ArduPilot via MAVLink SET_POSITION_TARGET_LOCAL_NED.
Adds basic safety limits and a deadman timeout.
"""

import sys
import time
from typing import Optional


def main() -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import TwistStamped
        from std_msgs.msg import String
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
            self.declare_parameter("timeout_sec", 0.5)
            self.declare_parameter("max_speed_xy", 1.0)
            self.declare_parameter("max_speed_z", 0.5)
            self.declare_parameter("max_yaw_rate", 0.5)
            self.mav: Optional[mavutil.mavfile] = None
            self.last_setpoint: Optional[TwistStamped] = None
            self.last_setpoint_time = 0.0
            self.connect_mavlink()
            self.sub = self.create_subscription(
                TwistStamped, "/control/setpoint", self.on_setpoint, 10
            )
            self.status_pub = self.create_publisher(String, "/mavlink/status", 10)
            rate = self.get_parameter("rate_hz").get_parameter_value().double_value
            self.timer = self.create_timer(1.0 / max(rate, 1.0), self.on_timer)
            self.status_timer = self.create_timer(1.0, self.on_status_timer)

        def connect_mavlink(self) -> None:
            url = self.get_parameter("mavlink_url").get_parameter_value().string_value
            self.get_logger().info(f"Connecting MAVLink: {url}")
            self.mav = mavutil.mavlink_connection(url)
            self.mav.wait_heartbeat(timeout=30)
            self.get_logger().info("MAVLink heartbeat received.")

        def on_setpoint(self, msg: TwistStamped) -> None:
            self.last_setpoint = msg
            self.last_setpoint_time = time.time()

        def on_timer(self) -> None:
            if self.mav is None:
                return
            timeout = self.get_parameter("timeout_sec").get_parameter_value().double_value
            if self.last_setpoint is None or (time.time() - self.last_setpoint_time) > timeout:
                self.send_setpoint(0.0, 0.0, 0.0, 0.0)
                return
            msg = self.last_setpoint
            max_xy = self.get_parameter("max_speed_xy").get_parameter_value().double_value
            max_z = self.get_parameter("max_speed_z").get_parameter_value().double_value
            max_yaw = self.get_parameter("max_yaw_rate").get_parameter_value().double_value
            vx = clamp(msg.twist.linear.x, -max_xy, max_xy)
            vy = clamp(msg.twist.linear.y, -max_xy, max_xy)
            vz = clamp(msg.twist.linear.z, -max_z, max_z)
            yaw_rate = clamp(msg.twist.angular.z, -max_yaw, max_yaw)
            self.send_setpoint(vx, vy, vz, yaw_rate)

        def on_status_timer(self) -> None:
            msg = String()
            msg.data = "mavlink:connected" if self.mav is not None else "mavlink:disconnected"
            self.status_pub.publish(msg)

        def send_setpoint(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
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
                float(vx),
                float(vy),
                float(vz),
                0.0,
                0.0,
                0.0,
                0.0,
                float(yaw_rate),
            )

    def clamp(value: float, min_v: float, max_v: float) -> float:
        return max(min_v, min(max_v, float(value)))

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
