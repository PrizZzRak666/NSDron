#!/usr/bin/env python3
"""PPO training entrypoint scaffold."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any, Dict

import yaml

from env import EnvConfig, SITLDroneEnv


def load_config(path: Path) -> Dict[str, Any]:
    data = yaml.safe_load(path.read_text())
    return data if isinstance(data, dict) else {}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("training/config.yaml"))
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
    except Exception as exc:
        print("stable-baselines3 not available.")
        print(f"Import error: {exc}")
        return 1

    cfg = load_config(args.config)
    env_cfg = EnvConfig(**cfg.get("env", {}))

    env = SITLDroneEnv(env_cfg)
    ppo_cfg = cfg.get("ppo", {})

    model = PPO("MlpPolicy", env, verbose=1, **ppo_cfg)
    model.learn(total_timesteps=10000)

    out_dir = Path("training/artifacts")
    out_dir.mkdir(parents=True, exist_ok=True)
    model.save(out_dir / "ppo_sitl")

    return 0


if __name__ == "__main__":
    sys.exit(main())
