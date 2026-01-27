#!/usr/bin/env python3
"""Smoke test for SITL MAVLink connectivity.

Checks heartbeat, reads one pose, and sends a zero-velocity setpoint.
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", default="udp:127.0.0.1:14550")
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    try:
        from pymavlink import mavutil
    except Exception as exc:
        print("pymavlink not available.")
        print(f"Import error: {exc}")
        return 1

    mav = mavutil.mavlink_connection(args.mavlink)
    hb = mav.wait_heartbeat(timeout=args.timeout)
    if hb is None:
        print("No heartbeat.")
        return 1
    print("Heartbeat ok.")

    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        10,
        1,
    )

    msg = mav.recv_match(type=["LOCAL_POSITION_NED", "ODOMETRY"], blocking=True, timeout=args.timeout)
    if msg is None:
        print("No pose received.")
        return 1
    print(f"Pose ok: {msg.get_type()}")

    mav.mav.set_position_target_local_ned_send(
        0,
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111000111,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    time.sleep(0.2)
    print("Setpoint sent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
