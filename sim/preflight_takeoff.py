#!/usr/bin/env python3
"""Basic MAVLink preflight: wait heartbeat, set GUIDED, arm, takeoff."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def wait_local_position(mav: mavutil.mavfile, timeout: float) -> float | None:
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = mav.recv_match(
            type=["LOCAL_POSITION_NED", "LOCAL_POSITION_NED_COV", "ODOMETRY"],
            blocking=True,
            timeout=1.0,
        )
        if msg is None:
            continue
        return float(getattr(msg, "z", 0.0))
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", default="udpin:0.0.0.0:14550")
    parser.add_argument("--alt", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    mav = mavutil.mavlink_connection(args.mavlink)
    deadline = time.time() + args.timeout
    hb = None
    while time.time() < deadline:
        hb = mav.wait_heartbeat(timeout=min(5.0, args.timeout))
        if hb is not None:
            break
        time.sleep(1.0)
    if hb is None:
        print("[error] No heartbeat")
        return 1

    try:
        mav.set_mode("GUIDED")
    except Exception:
        mav.mav.set_mode_send(
            mav.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
            | mavutil.mavlink.MAV_MODE_FLAG_GUIDED_ENABLED,
            4,
        )

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    mav.motors_armed_wait()

    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        float(abs(args.alt)),
    )

    target_z = -abs(args.alt)
    z = wait_local_position(mav, args.timeout)
    if z is None:
        print("[warn] No LOCAL_POSITION_NED after takeoff")
        return 0
    if z > target_z + 0.2:
        print(f"[warn] Takeoff not reached: z={z:.2f}")
        return 1
    print(f"[info] Takeoff ok: z={z:.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
