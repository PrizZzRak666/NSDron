"""Gym environment scaffold for ArduPilot SITL control.

Replace the stubs with real MAVLink integration and state parsing.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, Tuple

try:
    import gymnasium as gym
    import numpy as np
except Exception as exc:  # pragma: no cover - import guard
    gym = None
    np = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None


@dataclass
class EnvConfig:
    max_steps: int = 2000
    target_radius: float = 0.5
    max_speed: float = 1.0


class SITLDroneEnv(gym.Env):
    """Stub environment for SITL.

    Observation: [x, y, z, vx, vy, vz, tx, ty, tz]
    Action: [vx, vy, vz, yaw_rate]
    """

    metadata = {"render_modes": []}

    def __init__(self, config: EnvConfig | None = None) -> None:
        if _IMPORT_ERROR is not None:
            raise RuntimeError(f"Missing dependency: {_IMPORT_ERROR}")
        self.config = config or EnvConfig()
        self.step_count = 0

        obs_high = np.array([100, 100, 100, 10, 10, 10, 100, 100, 100], dtype=np.float32)
        act_high = np.array([self.config.max_speed] * 3 + [1.0], dtype=np.float32)

        self.observation_space = gym.spaces.Box(-obs_high, obs_high, dtype=np.float32)
        self.action_space = gym.spaces.Box(-act_high, act_high, dtype=np.float32)

    def reset(self, *, seed: int | None = None, options: Dict[str, Any] | None = None) -> Tuple[Any, Dict[str, Any]]:
        super().reset(seed=seed)
        self.step_count = 0
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        info: Dict[str, Any] = {}
        return obs, info

    def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        self.step_count += 1

        # TODO: send MAVLink setpoint, read pose, compute reward.
        obs = np.zeros(self.observation_space.shape, dtype=np.float32)
        reward = 0.0
        terminated = False
        truncated = self.step_count >= self.config.max_steps
        info: Dict[str, Any] = {}
        return obs, reward, terminated, truncated, info
