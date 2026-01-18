#!/usr/bin/env bash
set -euo pipefail

# Example launch sequence for ArduPilot SITL + Gazebo.
# Adjust paths to your local install.

echo "[info] Example launch sequence (review before running)"

cat <<'CMD'
# Terminal A (SITL):
# cd ~/ardupilot
# sim_vehicle.py -v ArduCopter -f gazebo-iris --console --map

# Terminal B (Gazebo):
# export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models:$GAZEBO_MODEL_PATH
# export GAZEBO_PLUGIN_PATH=~/ardupilot_gazebo/build:$GAZEBO_PLUGIN_PATH
# export GAZEBO_MODEL_PATH=/path/to/NSDron/sim/models:$GAZEBO_MODEL_PATH
# gazebo --verbose /path/to/NSDron/sim/worlds/iris_d455.world

# Terminal C (MAVLink check):
# mavproxy.py --master=udp:127.0.0.1:14550

# Terminal D (ROS2 camera topics):
# /path/to/NSDron/sim/check_camera_topics.sh
CMD
