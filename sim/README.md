# Simulation (ArduPilot SITL + Gazebo / gz sim)

This folder collects simulation setup notes and helper scripts.
The target is ArduPilot SITL with Gazebo (gz sim) and a camera model close to
Intel RealSense D455.

## Quick start (manual)
1) Install ArduPilot SITL + Gazebo (see `INSTALL.md`).
2) Launch SITL and Gazebo (gz sim) (see `start_sitl.sh`).
3) Confirm MAVLink connection and camera topics.

## Quick start (one command)
```
./sim/start_all.sh
```
Stops any running SITL/Gazebo, starts both, then launches training.
Use `./sim/start_all.sh --no-train` to start only SITL + Gazebo.

Stop everything:
```
./sim/stop_all.sh
```

## Files
- `setup_sitl.sh`: install and build steps (non-destructive; review before use).
- `start_sitl.sh`: example commands to launch SITL + Gazebo.
- `start_all.sh`: one-command launch (SITL + Gazebo + training).
- `stop_all.sh`: stop all related processes.
- `gazebo_notes.md`: camera model and sensor tuning notes.
- `INSTALL.md`: step-by-step setup checklist.
- `models/d455_camera`: basic RGB camera model.
- `worlds/iris_d455.world`: world file including Iris and camera.
- `check_camera_topics.sh`: helper to verify RGB/depth topics (gz or ROS2).

## Camera recording (ROS2)
Requires ROS2 + ros_gz_bridge + image_view:
```
sudo apt install -y ros-jazzy-ros-gz-bridge ros-jazzy-image-view
```

Then:
```
./sim/record_camera_ros2.sh
```
Images are saved to `~/NSDron/data/camera`.

## One-button dataset capture (frames + MAVLink)
Requires ROS2 + ros_gz_bridge + image_view and a running SITL + Gazebo.
```
./sim/record_dataset.sh
```
Outputs:
- `~/NSDron/data/dataset_YYYYMMDD_HHMMSS/frames/` (PNG frames)
- `~/NSDron/data/dataset_YYYYMMDD_HHMMSS/mavlink.csv` (pose/attitude/servo)

## Make video from frames
Install ffmpeg:
```
sudo apt install -y ffmpeg
```

Create video:
```
./sim/make_video.sh ~/NSDron/data/camera ~/NSDron/data/camera.mp4
```

## Notes
- Keep the camera intrinsics consistent between sim and real.
- Use the same coordinate frames (NED) as ArduPilot.
- Set geofence and failsafes in ArduPilot for safety.
- On WSL, prefer `gz sim -s` (headless) to avoid GUI/OpenGL crashes.
