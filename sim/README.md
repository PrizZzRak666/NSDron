# Simulation (ArduPilot SITL + Gazebo / gz sim)

This folder collects simulation setup notes and helper scripts.
The target is ArduPilot SITL with Gazebo (gz sim) and a camera model close to
Intel RealSense D455.

## Quick start (manual)
1) Install ArduPilot SITL + Gazebo (see `INSTALL.md`).
2) Launch SITL and Gazebo (gz sim) (see `start_sitl.sh`).
3) Confirm MAVLink connection and camera topics.

## Files
- `setup_sitl.sh`: install and build steps (non-destructive; review before use).
- `start_sitl.sh`: example commands to launch SITL + Gazebo.
- `gazebo_notes.md`: camera model and sensor tuning notes.
- `INSTALL.md`: step-by-step setup checklist.
- `models/d455_camera`: basic RGB camera model.
- `worlds/iris_d455.world`: world file including Iris and camera.
- `check_camera_topics.sh`: helper to verify RGB/depth topics (gz or ROS2).

## Notes
- Keep the camera intrinsics consistent between sim and real.
- Use the same coordinate frames (NED) as ArduPilot.
- Set geofence and failsafes in ArduPilot for safety.
- On WSL, prefer `gz sim -s` (headless) to avoid GUI/OpenGL crashes.
