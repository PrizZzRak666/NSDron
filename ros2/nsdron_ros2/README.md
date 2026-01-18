# nsdron_ros2 package

## Build
```
cd ros2
colcon build --packages-select nsdron_ros2
```

## Run
```
source install/setup.bash
ros2 launch nsdron_ros2 nsdron_launch.py
```

## Notes
- Requires rclpy, geometry_msgs, and pymavlink.
- Update MAVLink connection in `mavlink_bridge.py` parameters if needed.
