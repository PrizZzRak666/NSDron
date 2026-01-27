#!/usr/bin/env bash
set -euo pipefail

NSDRON_DIR="${NSDRON_DIR:-$HOME/NSDron}"
DATA_DIR="${DATA_DIR:-$NSDRON_DIR/data}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${OUT_DIR:-$DATA_DIR/dataset_$RUN_ID}"
FRAMES_DIR="$OUT_DIR/frames"
MAV_CSV="$OUT_DIR/mavlink.csv"
VENV_PY="${VENV_PY:-$NSDRON_DIR/.venv/bin/python}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[error] ros2 not found. Install ROS2 first."
  exit 1
fi

if [[ ! -x "$VENV_PY" ]]; then
  echo "[error] Python venv not found: $VENV_PY"
  exit 1
fi

mkdir -p "$FRAMES_DIR"

BRIDGE_CMD=(
  ros2 run ros_gz_bridge parameter_bridge
  /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image
  /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo
)

SAVER_CMD=(
  ros2 run image_view image_saver
  --ros-args -r image:=/camera/image_raw
  -p filename_format:="$FRAMES_DIR/frame%06d.png"
)

echo "[info] Dataset dir: $OUT_DIR"
echo "[info] Starting MAVLink logger: $MAV_CSV"

trap 'pkill -f ros_gz_bridge >/dev/null 2>&1 || true; pkill -f image_saver >/dev/null 2>&1 || true; pkill -f log_mavlink.py >/dev/null 2>&1 || true' EXIT

"$VENV_PY" "$NSDRON_DIR/training/log_mavlink.py" --mavlink tcp:127.0.0.1:5760 --out "$MAV_CSV" &

"${BRIDGE_CMD[@]}" &
sleep 2
"${SAVER_CMD[@]}"
