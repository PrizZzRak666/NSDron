#!/usr/bin/env bash
set -euo pipefail

# This script documents a typical ArduPilot SITL + Gazebo setup.
# Review and adapt before running on your system.

echo "[info] ArduPilot SITL + Gazebo setup guide"

echo "1) Install prerequisites (Ubuntu example):"
cat <<'CMD'
# sudo apt update
# sudo apt install -y git python3 python3-pip python3-venv build-essential
# sudo apt install -y gazebo libgazebo-dev
CMD

echo "2) Clone ArduPilot and set up environment:"
cat <<'CMD'
# git clone https://github.com/ArduPilot/ardupilot.git
# cd ardupilot
# git submodule update --init --recursive
# Tools/environment_install/install-prereqs-ubuntu.sh -y
# . ~/.profile
CMD

echo "3) Build SITL:"
cat <<'CMD'
# ./waf configure --board sitl
# ./waf copter
CMD

echo "4) (Optional) Install Gazebo plugin for ArduPilot"
cat <<'CMD'
# git clone https://github.com/ArduPilot/ardupilot_gazebo.git
# cd ardupilot_gazebo
# mkdir build && cd build
# cmake ..
# make -j$(nproc)
CMD

echo "[done] Review the commands above and run manually if desired."
