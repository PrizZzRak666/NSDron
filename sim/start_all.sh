#!/usr/bin/env bash
set -euo pipefail

NSDRON_DIR="${NSDRON_DIR:-$HOME/NSDron}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GZ_DIR="${ARDUPILOT_GZ_DIR:-$HOME/ardupilot/ardupilot_gazebo}"
WORLD="${WORLD:-$NSDRON_DIR/sim/worlds/iris_d455.world}"
VENV_PY="${VENV_PY:-$NSDRON_DIR/.venv/bin/python}"
LOG_DIR="${LOG_DIR:-$NSDRON_DIR/logs}"

mkdir -p "$LOG_DIR"

pkill -f arducopter >/dev/null 2>&1 || true
pkill -f "gz sim" >/dev/null 2>&1 || true
pkill -f mavproxy >/dev/null 2>&1 || true

SITL_LOG="$LOG_DIR/sitl.log"
GZ_LOG="$LOG_DIR/gz.log"

"$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
  --model gazebo-iris \
  --speedup 1 \
  --slave 0 \
  --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm" \
  --sim-address=127.0.0.1 -I0 >"$SITL_LOG" 2>&1 &

export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GZ_DIR/models:$NSDRON_DIR/sim/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GZ_DIR/build"

gz sim -s -v 4 "$WORLD" >"$GZ_LOG" 2>&1 &

sleep 3
echo "[info] SITL log: $SITL_LOG"
echo "[info] GZ log: $GZ_LOG"

if [[ "${1:-}" == "--no-train" ]]; then
  echo "[info] SITL + Gazebo started. Run training manually."
  exit 0
fi

if [[ ! -x "$VENV_PY" ]]; then
  echo "[error] Python venv not found: $VENV_PY"
  exit 1
fi

"$VENV_PY" "$NSDRON_DIR/training/train.py" --config "$NSDRON_DIR/training/config.yaml"
