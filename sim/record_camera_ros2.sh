#!/usr/bin/env bash
set -euo pipefail

NSDRON_DIR="${NSDRON_DIR:-$HOME/NSDron}"
OUT_DIR="${1:-$NSDRON_DIR/data/camera}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 not found. Install ROS2 first."
  exit 1
fi

mkdir -p "$OUT_DIR"

BRIDGE_CMD=(
  ros2 run ros_gz_bridge parameter_bridge
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
  /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
)

SAVER_CMD=(
  ros2 run image_view image_saver
  --ros-args -r image:=/camera/image_raw
  -p filename_format:="$OUT_DIR/frame%06d.png"
)

echo "[info] Saving images to: $OUT_DIR"
echo "[info] Bridge: ${BRIDGE_CMD[*]}"
echo "[info] Saver: ${SAVER_CMD[*]}"

trap 'pkill -f ros_gz_bridge >/dev/null 2>&1 || true' EXIT

"${BRIDGE_CMD[@]}" &
sleep 2
"${SAVER_CMD[@]}"
