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

## Smoke test
```
python training/smoke_test.py --mavlink udp:127.0.0.1:14550
```

## Notes
- The environment uses `LOCAL_POSITION_NED` for pose.
- Observations: local pose, target pose, velocity.
- Actions: velocity setpoints (vx, vy, vz, yaw_rate).
- If `auto_guided`/`auto_arm` are enabled in `config.yaml`, the env will set
  GUIDED mode and arm automatically.
