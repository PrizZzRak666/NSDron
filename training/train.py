#!/usr/bin/env python3
"""PPO training entrypoint scaffold."""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
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
    parser.add_argument("--mavlink", type=str, default=None)
    parser.add_argument("--device", type=str, default=None)
    parser.add_argument("--log-dir", type=Path, default=None)
    parser.add_argument("--total-timesteps", type=int, default=None)
    args = parser.parse_args()

    try:
        from stable_baselines3 import PPO
        from stable_baselines3.common.callbacks import CheckpointCallback
        from stable_baselines3.common.logger import configure as sb3_configure
        from stable_baselines3.common.monitor import Monitor
        from stable_baselines3.common.vec_env import DummyVecEnv, VecMonitor
    except Exception as exc:
        print("stable-baselines3 not available.")
        print(f"Import error: {exc}")
        return 1

    cfg = load_config(args.config)
    env_cfg = EnvConfig(**cfg.get("env", {}))
    if args.mavlink:
        env_cfg.mavlink_url = args.mavlink

    train_cfg = cfg.get("train", {})
    base_log_dir = args.log_dir or Path(train_cfg.get("log_dir", "training/logs"))
    run_id = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir = base_log_dir / f"run_{run_id}"
    log_dir.mkdir(parents=True, exist_ok=True)

    env_cfg.log_dir = str(log_dir)
    env = SITLDroneEnv(env_cfg)
    env = Monitor(env, str(log_dir / "monitor"))
    env = DummyVecEnv([lambda: env])
    env = VecMonitor(env, filename=str(log_dir / "vecmonitor.csv"))

    ppo_cfg = cfg.get("ppo", {})
    if args.device:
        ppo_cfg["device"] = args.device
    elif "device" in train_cfg:
        ppo_cfg["device"] = train_cfg["device"]

    model = PPO("MlpPolicy", env, verbose=1, **ppo_cfg)
    logger = sb3_configure(str(log_dir), ["stdout", "csv", "tensorboard"])
    model.set_logger(logger)

    total_timesteps = int(args.total_timesteps or train_cfg.get("total_timesteps", 10000))
    save_freq = int(train_cfg.get("save_freq", max(total_timesteps // 5, 1000)))
    checkpoint_cb = CheckpointCallback(
        save_freq=save_freq, save_path=str(log_dir / "checkpoints"), name_prefix="ppo"
    )

    model.learn(total_timesteps=total_timesteps, callback=checkpoint_cb)

    model.save(log_dir / "ppo_sitl")
    print(f"[info] Saved model to {log_dir / 'ppo_sitl.zip'}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
