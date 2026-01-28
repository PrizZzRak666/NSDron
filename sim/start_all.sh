#!/usr/bin/env bash
set -euo pipefail

NSDRON_DIR="${NSDRON_DIR:-$HOME/NSDron}"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_GZ_DIR="${ARDUPILOT_GZ_DIR:-$HOME/ardupilot/ardupilot_gazebo}"
WORLD="${WORLD:-$NSDRON_DIR/sim/worlds/iris_d455.world}"
VENV_PY="${VENV_PY:-$NSDRON_DIR/.venv/bin/python}"
LOG_DIR="${LOG_DIR:-$NSDRON_DIR/logs}"
TRAIN_LOG="${TRAIN_LOG:-$LOG_DIR/train.log}"
SITL_SIM_PORT_IN="${SITL_SIM_PORT_IN:-9003}"
SITL_SIM_PORT_OUT="${SITL_SIM_PORT_OUT:-9002}"
MAVLINK_TCP_PORT="${MAVLINK_TCP_PORT:-5762}"
SERIAL1_URL="${SERIAL1_URL:-tcp:0.0.0.0:${MAVLINK_TCP_PORT}}"
SERIAL0_URL="${SERIAL0_URL:-udpclient:127.0.0.1:14560}"

mkdir -p "$LOG_DIR"

pkill -f arducopter >/dev/null 2>&1 || true
pkill -f "gz sim" >/dev/null 2>&1 || true
pkill -f mavproxy >/dev/null 2>&1 || true

SITL_LOG="$LOG_DIR/sitl.log"
GZ_LOG="$LOG_DIR/gz.log"

DEFAULTS="$ARDUPILOT_DIR/Tools/autotest/default_params/copter.parm,$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm"
if [[ -f "$NSDRON_DIR/sim/serial1_mavlink.parm" ]]; then
  DEFAULTS="$DEFAULTS,$NSDRON_DIR/sim/serial1_mavlink.parm"
fi

MODEL_SDF="$ARDUPILOT_GZ_DIR/models/iris_with_ardupilot/model.sdf"
if [[ -f "$MODEL_SDF" ]]; then
  FDM_IN=$(grep -oP '<fdm_port_in>\K[0-9]+' "$MODEL_SDF" | head -n1 || true)
  FDM_OUT=$(grep -oP '<fdm_port_out>\K[0-9]+' "$MODEL_SDF" | head -n1 || true)
  # SITL sim-port-out must match plugin fdm_port_in, and sim-port-in must match fdm_port_out.
  if [[ -n "${FDM_IN:-}" ]]; then
    SITL_SIM_PORT_OUT="$FDM_IN"
  fi
  if [[ -n "${FDM_OUT:-}" ]]; then
    SITL_SIM_PORT_IN="$FDM_OUT"
  fi
fi

"$ARDUPILOT_DIR/build/sitl/bin/arducopter" \
  --model JSON \
  --speedup 1 \
  --slave 0 \
  --defaults "$DEFAULTS" \
  --sim-address=127.0.0.1 --sim-port-in "$SITL_SIM_PORT_IN" --sim-port-out "$SITL_SIM_PORT_OUT" \
  --serial0 "$SERIAL0_URL" \
  --serial1 "$SERIAL1_URL" \
  -I0 >"$SITL_LOG" 2>&1 &

export GZ_SIM_RESOURCE_PATH="$ARDUPILOT_GZ_DIR/models:$NSDRON_DIR/sim/models"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GZ_DIR/build"

gz sim -s -v 4 "$WORLD" >"$GZ_LOG" 2>&1 &

sleep 3

if command -v ss >/dev/null 2>&1; then
  for _ in {1..20}; do
    if ss -lnt 2>/dev/null | grep -q ":${MAVLINK_TCP_PORT}"; then
      break
    fi
    sleep 1
  done
fi
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
