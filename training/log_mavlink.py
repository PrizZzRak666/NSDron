#!/usr/bin/env python3
"""Log MAVLink pose/attitude/servo outputs to CSV for dataset alignment."""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path


def _nan() -> float:
    return float("nan")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mavlink", default="tcp:127.0.0.1:5760")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    try:
        from pymavlink import mavutil
    except Exception as exc:
        print("pymavlink not available.")
        print(f"Import error: {exc}")
        return 1

    args.out.parent.mkdir(parents=True, exist_ok=True)

    mav = mavutil.mavlink_connection(args.mavlink)
    hb = mav.wait_heartbeat(timeout=15)
    if hb is None:
        print("No heartbeat.")
        return 1
    print("Heartbeat ok.")

    # Request position/attitude/servo streams
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        10,
        1,
    )
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        10,
        1,
    )
    mav.mav.request_data_stream_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
        10,
        1,
    )

    header = [
        "time_unix",
        "time_boot_ms",
        "pos_x",
        "pos_y",
        "pos_z",
        "vel_x",
        "vel_y",
        "vel_z",
        "roll",
        "pitch",
        "yaw",
        "rollspeed",
        "pitchspeed",
        "yawspeed",
        "servo1",
        "servo2",
        "servo3",
        "servo4",
        "target_vx",
        "target_vy",
        "target_vz",
    ]

    last_att = None
    last_servo = None
    last_target = None

    with args.out.open("w", newline="") as fp:
        writer = csv.writer(fp)
        writer.writerow(header)
        fp.flush()

        while True:
            msg = mav.recv_match(
                type=[
                    "LOCAL_POSITION_NED",
                    "ODOMETRY",
                    "ATTITUDE",
                    "SERVO_OUTPUT_RAW",
                    "POSITION_TARGET_LOCAL_NED",
                    "POS_TARGET_LOCAL_NED",
                ],
                blocking=True,
                timeout=args.timeout,
            )
            if msg is None:
                continue

            mtype = msg.get_type()
            if mtype == "ATTITUDE":
                last_att = msg
                continue
            if mtype == "SERVO_OUTPUT_RAW":
                last_servo = msg
                continue
            if mtype in ("POSITION_TARGET_LOCAL_NED", "POS_TARGET_LOCAL_NED"):
                last_target = msg
                continue

            if mtype not in ("LOCAL_POSITION_NED", "ODOMETRY"):
                continue

            if mtype == "LOCAL_POSITION_NED":
                pos_x, pos_y, pos_z = msg.x, msg.y, msg.z
                vel_x, vel_y, vel_z = msg.vx, msg.vy, msg.vz
                time_boot_ms = msg.time_boot_ms
            else:
                pos_x, pos_y, pos_z = msg.x, msg.y, msg.z
                vel_x, vel_y, vel_z = msg.vx, msg.vy, msg.vz
                time_boot_ms = getattr(msg, "time_usec", 0) // 1000

            if last_att is None:
                roll = pitch = yaw = _nan()
                rollspeed = pitchspeed = yawspeed = _nan()
            else:
                roll = last_att.roll
                pitch = last_att.pitch
                yaw = last_att.yaw
                rollspeed = last_att.rollspeed
                pitchspeed = last_att.pitchspeed
                yawspeed = last_att.yawspeed

            if last_servo is None:
                servo1 = servo2 = servo3 = servo4 = _nan()
            else:
                servo1 = last_servo.servo1_raw
                servo2 = last_servo.servo2_raw
                servo3 = last_servo.servo3_raw
                servo4 = last_servo.servo4_raw

            if last_target is None:
                target_vx = target_vy = target_vz = _nan()
            else:
                target_vx = getattr(last_target, "vx", _nan())
                target_vy = getattr(last_target, "vy", _nan())
                target_vz = getattr(last_target, "vz", _nan())

            writer.writerow(
                [
                    time.time(),
                    int(time_boot_ms),
                    float(pos_x),
                    float(pos_y),
                    float(pos_z),
                    float(vel_x),
                    float(vel_y),
                    float(vel_z),
                    float(roll),
                    float(pitch),
                    float(yaw),
                    float(rollspeed),
                    float(pitchspeed),
                    float(yawspeed),
                    float(servo1),
                    float(servo2),
                    float(servo3),
                    float(servo4),
                    float(target_vx) if not math.isnan(target_vx) else _nan(),
                    float(target_vy) if not math.isnan(target_vy) else _nan(),
                    float(target_vz) if not math.isnan(target_vz) else _nan(),
                ]
            )
            fp.flush()


if __name__ == "__main__":
    raise SystemExit(main())
