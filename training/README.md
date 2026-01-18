# Training (PPO scaffold)

This is a minimal scaffold for a Gym-style environment and PPO training loop.
It does not connect to SITL yet; use it as a template.

## Dependencies
- Python 3.10+
- gymnasium
- stable-baselines3

## Usage
1) Create a virtualenv and install dependencies.
2) Implement the real environment in `env.py`.
3) Run training:

```
python training/train.py --config training/config.yaml
```

## Notes
- The environment should wrap ArduPilot SITL via MAVLink.
- Observations: local pose, target pose, velocity, optional camera features.
- Actions: velocity setpoints (vx, vy, vz, yaw_rate).
