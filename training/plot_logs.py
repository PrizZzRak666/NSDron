#!/usr/bin/env python3
"""Plot training logs (monitor/vecmonitor) to PNG files."""

from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", type=Path, required=True)
    args = parser.parse_args()

    log_dir = args.log_dir
    monitor = log_dir / "monitor.csv"
    vecmonitor = log_dir / "vecmonitor.csv"

    if not monitor.exists() and not vecmonitor.exists():
        print(f"No monitor logs found in {log_dir}")
        return 1

    data_frames = []
    if monitor.exists():
        df = pd.read_csv(monitor, comment="#")
        df["source"] = "monitor"
        data_frames.append(df)
    if vecmonitor.exists():
        df = pd.read_csv(vecmonitor)
        df["source"] = "vecmonitor"
        data_frames.append(df)

    df_all = pd.concat(data_frames, ignore_index=True)

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"matplotlib not available: {exc}")
        return 1

    if "r" in df_all.columns:
        plt.figure()
        plt.plot(df_all["r"].values)
        plt.title("Episode reward")
        plt.xlabel("Episode")
        plt.ylabel("Reward")
        plt.tight_layout()
        plt.savefig(log_dir / "reward.png", dpi=150)

    if "l" in df_all.columns:
        plt.figure()
        plt.plot(df_all["l"].values)
        plt.title("Episode length")
        plt.xlabel("Episode")
        plt.ylabel("Steps")
        plt.tight_layout()
        plt.savefig(log_dir / "length.png", dpi=150)

    print(f"[info] Plots saved to {log_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
