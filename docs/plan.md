# Plan (MVP)

## Phase 1: Simulation bring-up
- Install ArduPilot SITL + Gazebo.
- Validate manual flight and MAVLink connectivity.
- Add D455 camera model and confirm image topics.

## Phase 2: Localization
- Integrate ORB-SLAM3 in sim.
- Publish `/slam/pose` at >= 20 Hz.
- Add pose-loss detection and fallback behavior.

## Phase 3: Control baseline
- Implement MAVLink setpoint node.
- Fly a square with deterministic controller using `/slam/pose`.
- Log state and actions for training data.

## Phase 4: RL training
- Build Gym-style environment wrapping SITL.
- Train PPO to track waypoints using local pose + camera (optional).
- Validate against baseline controller.

## Phase 5: Real-world deployment
- Jetson setup: ROS2 + SLAM + MAVLink.
- Calibrate camera, verify pose stability.
- Test in a safe environment with geofence and manual override.

