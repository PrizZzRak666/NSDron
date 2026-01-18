# ROS2 interfaces (proposed)

Topics:
- /slam/pose: geometry_msgs/PoseStamped
  - Source: ORB-SLAM3 or VIO node
- /state/local_pose: geometry_msgs/PoseStamped
  - Source: pose_bridge
- /mission/target: geometry_msgs/PoseStamped
  - Source: mission_node
- /control/setpoint: geometry_msgs/TwistStamped
  - Source: control_node
- /mavlink/status: std_msgs/String
  - Source: mavlink_bridge

Frames:
- Use NED or ENU consistently. ArduPilot uses NED.
- Document transforms if SLAM provides ENU.

Rate:
- Pose: >= 20 Hz
- Setpoints: 10-20 Hz
