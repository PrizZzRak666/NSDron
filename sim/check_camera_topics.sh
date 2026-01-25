#!/usr/bin/env bash
set -euo pipefail

if command -v gz >/dev/null 2>&1; then
  echo "[info] Listing gz camera topics"
  gz topic -l | grep -E "/camera" || true
  echo
  echo "[info] Expected topics (gz sim):"
  echo "  /camera/image_raw"
  echo "  /camera/depth/image_raw"
  echo
  echo "[info] To preview a topic:"
  echo "  gz topic -e -t /camera/image_raw --once"
  echo "  gz topic -e -t /camera/depth/image_raw --once"
  exit 0
fi

if command -v ros2 >/dev/null 2>&1; then
  echo "[info] Listing ROS2 camera topics"
  ros2 topic list | grep -E "/camera" || true
  echo
  echo "[info] Expected topics (ROS2 bridge):"
  echo "  /camera/image_raw"
  echo "  /camera/camera_info"
  echo "  /camera/depth/image_raw"
  echo "  /camera/depth/camera_info"
  echo
  echo "[info] To preview a topic:"
  echo "  ros2 topic echo /camera/image_raw --once"
  echo "  ros2 topic echo /camera/depth/image_raw --once"
  exit 0
fi

echo "Neither gz nor ros2 CLI found. Install Gazebo (gz sim) or ROS2."
exit 1
