# Training (PPO scaffold)

This is a minimal Gym-style environment and PPO training loop that connects
to ArduPilot SITL via MAVLink.

## Dependencies
- Python 3.10+
- gymnasium
- stable-baselines3
- pymavlink

## Usage
1) Create a virtualenv and install dependencies.
2) Start ArduPilot SITL + Gazebo. If `auto_guided`/`auto_arm` are disabled,
   arm the vehicle in GUIDED mode.
3) Run training:

```
python training/train.py --config training/config.yaml
```

### Useful overrides
```
python training/train.py --config training/config.yaml --device cpu
python training/train.py --config training/config.yaml --mavlink tcp:127.0.0.1:5760
python training/train.py --config training/config.yaml --total-timesteps 2000
python training/train.py --config training/config.yaml --log-dir training/logs
```

Logs and checkpoints are written to `training/logs/run_YYYYMMDD_HHMMSS/`.

### Trajectory CSV
If `env.log_trajectory: true`, a per-step CSV is saved to:
`training/logs/run_YYYYMMDD_HHMMSS/trajectory.csv`

### MAVLink logger (pose/attitude/servo)
```
python training/log_mavlink.py --mavlink tcp:127.0.0.1:5760 --out training/logs/mavlink.csv
```

### Plot logs
```
python training/plot_logs.py --log-dir training/logs/run_YYYYMMDD_HHMMSS
```

## Smoke test
```
python training/smoke_test.py --mavlink tcp:127.0.0.1:5760
```

## Notes
- The environment uses `LOCAL_POSITION_NED` for pose.
- Observations: local pose, target pose, velocity.
- Actions: velocity setpoints (vx, vy, vz, yaw_rate).
- If `auto_guided`/`auto_arm` are enabled in `config.yaml`, the env will set
  GUIDED mode and arm automatically.
