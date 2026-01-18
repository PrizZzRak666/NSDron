# Architecture (MVP)

Goal: takeoff, fly 4 waypoints using vision-based localization (no GPS/markers),
then return to home. Train in simulation and deploy to a real ArduPilot drone
with Jetson companion.

## High-level flow
1) Camera stream -> VIO/SLAM -> local pose (x, y, z, yaw).
2) Mission logic -> waypoint sequence -> local target pose.
3) Policy (PPO) -> velocity/attitude setpoints.
4) MAVLink -> ArduPilot -> motors.

## Modules
- Perception
  - Intel RealSense D455.
  - ORB-SLAM3 provides local pose; publishes to ROS2.
- State
  - Fuses SLAM pose with IMU for smoothing.
  - Maintains home frame and waypoint progress.
- Control
  - PPO policy consumes local pose + target + camera features (optional).
  - Outputs velocity setpoints at 10-20 Hz.
- Safety
  - Geofence and altitude limits in ArduPilot.
  - Failsafe to hover/RTL on pose loss.

## Interfaces
- ROS2 topics:
  - `/camera/*` from RealSense.
  - `/slam/pose` local pose from ORB-SLAM3.
  - `/mission/target` next waypoint.
  - `/control/setpoint` velocity or attitude.
- MAVLink:
  - `SET_POSITION_TARGET_LOCAL_NED` for velocity setpoints.

## Sim-to-real alignment
- Use Gazebo with ArduPilot SITL and a camera model matching D455 intrinsics.
- Calibrate camera/IMU and keep the same frame conventions in sim and real.
- Train with domain randomization: lighting, textures, and motion noise.

