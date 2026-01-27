#!/usr/bin/env bash
set -euo pipefail

pkill -f arducopter >/dev/null 2>&1 || true
pkill -f "gz sim" >/dev/null 2>&1 || true
pkill -f mavproxy >/dev/null 2>&1 || true
pkill -f "NSDron/training/train.py" >/dev/null 2>&1 || true
pkill -f tensorboard >/dev/null 2>&1 || true
echo "[info] stopped SITL, Gazebo, MAVProxy, training, TensorBoard"
