#!/usr/bin/env bash
set -euo pipefail

IN_DIR="${1:-$HOME/NSDron/data/camera}"
OUT_FILE="${2:-$HOME/NSDron/data/camera.mp4}"

if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "[error] ffmpeg not found. Install with: sudo apt install -y ffmpeg"
  exit 1
fi

if [[ ! -d "$IN_DIR" ]]; then
  echo "[error] input dir not found: $IN_DIR"
  exit 1
fi

ffmpeg -y -framerate 30 -i "$IN_DIR/frame%06d.png" \
  -c:v libx264 -pix_fmt yuv420p "$OUT_FILE"

echo "[info] Video saved to: $OUT_FILE"
