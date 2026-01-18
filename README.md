# NSDron

MVP: autonomous flight without GPS/markers. Drone takes off, flies 4 waypoints using
vision-based localization, then returns home. Training happens in simulation and
deploys to a real ArduPilot drone with a Jetson companion.

## Stack (MVP)
- Sim: Gazebo + ArduPilot SITL
- Localization: ORB-SLAM3 (RealSense D455 stereo/depth)
- Control: MAVLink setpoints
- Orchestration: ROS2
- Learning: PPO baseline (vision + state)

## Repo layout
- `docs/`: architecture, plan, and hardware notes
- `sim/`: simulation configs and scripts
- `ros2/`: ROS2 nodes (SLAM bridge, state, control)
- `training/`: RL training code and configs
- `src/`: shared utilities and wrappers

## First milestones
1) Bring up ArduPilot SITL + Gazebo and confirm manual flight.
2) Integrate RealSense + ORB-SLAM3 in sim and publish local pose.
3) Implement MAVLink setpoint node and fly a square in sim.
4) Train PPO to follow 4 waypoints using local pose and camera.
5) Deploy the same stack on Jetson and validate on a real drone.

See `docs/architecture.md` and `docs/plan.md` for details.
