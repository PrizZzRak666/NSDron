#!/usr/bin/env bash
set -euo pipefail

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 CLI not found. Install ROS2 and source its setup.bash."
  exit 1
fi

echo "[info] Listing camera-related topics"
ros2 topic list | grep -E "/camera" || true

echo
echo "[info] Expected topics (names can vary):"
echo "  /camera/image_raw"
echo "  /camera/camera_info"
echo "  /camera/depth/image_raw"
echo "  /camera/depth/camera_info"
echo
echo "[info] To preview a topic:"
echo "  ros2 topic echo /camera/image_raw --once"
echo "  ros2 topic echo /camera/depth/image_raw --once"
