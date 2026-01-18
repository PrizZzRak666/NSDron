# ROS2 nodes (MVP)

This folder provides minimal node scaffolds to bridge SLAM pose, mission
waypoints, and MAVLink setpoints.

## Nodes
- `pose_bridge.py`: converts SLAM/VIO pose to a standardized local frame.
- `mission_node.py`: serves the 4-waypoint mission and return-to-home logic.
- `control_node.py`: converts target + state into MAVLink setpoints.
- `mavlink_bridge.py`: sends setpoints to ArduPilot via MAVLink.

## Topics (proposed)
- `/slam/pose` (geometry_msgs/PoseStamped)
- `/state/local_pose` (geometry_msgs/PoseStamped)
- `/mission/target` (geometry_msgs/PoseStamped)
- `/control/setpoint` (geometry_msgs/TwistStamped)

## Usage
These are scaffolds. A working ROS2 Python package exists under
`ros2/nsdron_ros2` with a launch file for all nodes.
