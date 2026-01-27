#!/usr/bin/env bash
set -euo pipefail

NSDRON_DIR="${NSDRON_DIR:-$HOME/NSDron}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GZ_DIR="${ARDUPILOT_GZ_DIR:-$HOME/ardupilot/ardupilot_gazebo}"
EMPTY_WORLD="${EMPTY_WORLD:-/opt/ros/jazzy/opt/gz_sim_vendor/share/gz/gz-sim8/worlds/empty.sdf}"
WORLD_NAME="${WORLD_NAME:-empty}"
VENV_PY="${VENV_PY:-$NSDRON_DIR/.venv/bin/python}"
LOG_DIR="${LOG_DIR:-$NSDRON_DIR/logs}"
TRAIN_LOG="${TRAIN_LOG:-$LOG_DIR/train.log}"
FUEL_PATH="${FUEL_PATH:-$HOME/.gz/fuel/fuel.gazebosim.org/OpenRobotics/models}"
GZ_BIN="${GZ_BIN:-/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz}"

mkdir -p "$LOG_DIR"

pkill -f arducopter >/dev/null 2>&1 || true
pkill -f "gz sim" >/dev/null 2>&1 || true
pkill -f mavproxy >/dev/null 2>&1 || true

SITL_LOG="$LOG_DIR/sitl.log"
GZ_LOG="$LOG_DIR/gz.log"

if [[ ! -x "$GZ_BIN" ]]; then
  GZ_BIN="$(command -v gz || true)"
fi
if [[ -z "${GZ_BIN:-}" ]]; then
  echo "[error] gz not found in PATH and GZ_BIN is invalid"
  exit 1
fi

"$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
  --model gazebo-iris \
  --speedup 1 \
  --slave 0 \
  --defaults "$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm" \
  --sim-address=127.0.0.1 --sim-port-in 9003 --sim-port-out 9002 -I0 >"$SITL_LOG" 2>&1 &

export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GZ_DIR/models:$NSDRON_DIR/sim/models"
if [[ -d "$FUEL_PATH" ]]; then
  export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$FUEL_PATH"
fi
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GZ_DIR/build"

"$GZ_BIN" sim -v 4 -r "$EMPTY_WORLD" --force-version 8 >"$GZ_LOG" 2>&1 &

for _ in {1..30}; do
  if "$GZ_BIN" service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/create"; then
    break
  fi
  sleep 1
done

if ! "$GZ_BIN" service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/create"; then
  echo "[error] Gazebo world '${WORLD_NAME}' not ready. Check $GZ_LOG"
  exit 1
fi

"$GZ_BIN" service -s "/world/${WORLD_NAME}/create" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'sdf_filename: "'"$ARDUPILOT_GZ_DIR"'/models/iris_with_ardupilot/model.sdf", name: "iris_with_ardupilot", pose: {position: {z: 1}}' >/dev/null 2>&1 || true

"$GZ_BIN" service -s "/world/${WORLD_NAME}/create" \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 3000 \
  --req 'sdf_filename: "'"$NSDRON_DIR"'/sim/models/d455_camera/model.sdf", name: "d455_camera", pose: {position: {x: 2, z: 1}}' >/dev/null 2>&1 || true

echo "[info] SITL log: $SITL_LOG"
echo "[info] GZ log: $GZ_LOG"
echo "[info] Train log: $TRAIN_LOG"

if [[ "${1:-}" == "--no-train" ]]; then
  echo "[info] SITL + Gazebo started. Run training manually."
  exit 0
fi

if [[ ! -x "$VENV_PY" ]]; then
  echo "[error] Python venv not found: $VENV_PY"
  exit 1
fi

"$VENV_PY" "$NSDRON_DIR/training/train.py" --config "$NSDRON_DIR/training/config.yaml" >"$TRAIN_LOG" 2>&1 &
echo "[info] Training PID: $!"
