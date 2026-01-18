# Gazebo camera notes

Goal: approximate Intel RealSense D455.

Suggested starting parameters:
- Resolution: 640x480 or 848x480
- FPS: 30
- HFOV: ~86 deg
- Near/Far: 0.3 / 20.0

Tips:
- Keep intrinsics aligned with your real camera profile.
- Add mild noise for sim-to-real robustness.
- Confirm the camera frame name matches ROS2 nodes.
- If using depth, expose both RGB and depth topics.
