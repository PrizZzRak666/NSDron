#!/usr/bin/env bash
set -euo pipefail

# Example launch sequence for ArduPilot SITL + gz sim.
# Adjust paths to your local install.

echo "[info] Example launch sequence (review before running)"

cat <<'CMD'
# Terminal A (SITL):
# cd ~/ardupilot
# sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

# Terminal B (gz sim, headless):
# GZ_SIM_RESOURCE_PATH=~/ardupilot_gazebo/models:/path/to/NSDron/sim/models \
# GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build \
# gz sim -s -v 4 /path/to/NSDron/sim/worlds/iris_d455.world

# Terminal C (MAVLink check):
# mavproxy.py --master=udp:127.0.0.1:14550

# Terminal D (camera topics):
# /path/to/NSDron/sim/check_camera_topics.sh
CMD
