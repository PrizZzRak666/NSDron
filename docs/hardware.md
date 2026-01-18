# Hardware notes

## Recommended starter hardware
- Companion: NVIDIA Jetson (Orin Nano or Xavier NX).
- Camera: Intel RealSense D455 (preferred) or D435i.
- Autopilot: ArduPilot-compatible FC (Pixhawk class).

## Why D455
- Better depth range and outdoor robustness.
- Widely supported ROS2 drivers.

## Alternatives
- OAK-D: strong onboard processing but more integration effort.
- Stereo-only camera with VIO (ORB-SLAM3) if depth is not needed.

