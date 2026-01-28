#!/usr/bin/env bash
set -euo pipefail

# Example launch sequence for ArduPilot SITL + gz sim.
# Adjust paths to your local install.

echo "[info] Example launch sequence (review before running)"

cat <<'CMD'
# Terminal A (SITL):
# cd ~/ardupilot
# sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map
#
# Note: SITL default serial mapping:
# - SERIAL0 (console): tcp:5760
# - SERIAL1 (MAVLink): tcp:5762
#
# Terminal B (gz sim, headless):
# GZ_SIM_RESOURCE_PATH=~/ardupilot_gazebo/models:/path/to/NSDron/sim/models \
# GZ_SIM_SYSTEM_PLUGIN_PATH=~/ardupilot_gazebo/build \
# gz sim -s -v 4 /path/to/NSDron/sim/worlds/iris_d455.world
#
# Terminal C (MAVLink check over TCP):
# mavproxy.py --master=tcp:127.0.0.1:5762
#
# Optional (UDP): only if SITL started with
#   --serial1 udpclient:127.0.0.1:14550
# then:
# mavproxy.py --master=udp:127.0.0.1:14550
#
# Note: if you launch arducopter directly, SERIAL0 (console) can block startup.
# You can disable the TCP console with:
#   --serial0 udpclient:127.0.0.1:14560
#
# Terminal D (camera topics):
# /path/to/NSDron/sim/check_camera_topics.sh
CMD
