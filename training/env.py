"""Gym environment for ArduPilot SITL control via MAVLink."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

try:
    import gymnasium as gym
    import numpy as np
except Exception as exc:  # pragma: no cover - import guard
    gym = None
    np = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None

try:
    from pymavlink import mavutil
except Exception as exc:  # pragma: no cover - import guard
    mavutil = None
    _MAVLINK_ERROR = exc
else:
    _MAVLINK_ERROR = None


@dataclass
class EnvConfig:
    max_steps: int = 2000
    target_radius: float = 0.5
    max_speed: float = 1.0
    mavlink_url: str = "udp:127.0.0.1:14550"
    pose_timeout: float = 0.5
    action_rate_hz: float = 10.0
    auto_guided: bool = True
    guided_mode_id: int = 4
    auto_arm: bool = True
    arm_timeout: float = 5.0
    disarm_on_close: bool = False
    waypoints: List[List[float]] = field(
        default_factory=lambda: [
            [2.0, 0.0, -2.0],
            [2.0, 2.0, -2.0],
            [0.0, 2.0, -2.0],
            [0.0, 0.0, -2.0],
        ]
    )


class MavlinkClient:
    """Minimal MAVLink client for local pose and velocity setpoints."""

    def __init__(self, url: str, stream_rate_hz: float = 20.0) -> None:
        if _MAVLINK_ERROR is not None:
            raise RuntimeError(f"Missing dependency: {_MAVLINK_ERROR}")
        self.mav = mavutil.mavlink_connection(url)
        self.mav.wait_heartbeat(timeout=30)
        self.request_streams(stream_rate_hz)

    def request_streams(self, rate_hz: float) -> None:
        rate = max(int(rate_hz), 1)
        self.mav.mav.request_data_stream_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            rate,
            1,
        )

    def set_mode_guided(self, guided_mode_id: int) -> None:
        base_mode = mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        self.mav.mav.set_mode_send(self.mav.target_system, base_mode, guided_mode_id)

    def arm(self, timeout: float = 5.0) -> bool:
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1,  # arm
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return self._wait_for_arm(True, timeout)

    def disarm(self, timeout: float = 5.0) -> bool:
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,  # disarm
            0,
            0,
            0,
            0,
            0,
            0,
        )
        return self._wait_for_arm(False, timeout)

    def _wait_for_arm(self, armed: bool, timeout: float) -> bool:
        deadline = time.time() + timeout
        while time.time() < deadline:
            msg = self.mav.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if msg is None:
                continue
            is_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if is_armed == armed:
                return True
        return False

    def read_state(self, timeout: float) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        msg = self.mav.recv_match(
            type=["LOCAL_POSITION_NED", "LOCAL_POSITION_NED_COV", "ODOMETRY"],
            blocking=True,
            timeout=timeout,
        )
        if msg is None:
            return None
        msg_type = msg.get_type()
        if msg_type == "ODOMETRY":
            pos = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
            vel = np.array([msg.vx, msg.vy, msg.vz], dtype=np.float32)
        else:
            pos = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
            vel = np.array([msg.vx, msg.vy, msg.vz], dtype=np.float32)
        return pos, vel

    def send_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        self.mav.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,  # ignore pos, accel, yaw
            0.0,
            0.0,
            0.0,
            float(vx),
            float(vy),
            float(vz),
            0.0,
            0.0,
            0.0,
            0.0,
            float(yaw_rate),
        )


class SITLDroneEnv(gym.Env):
    """SITL environment using local NED pose and velocity control."""

    metadata = {"render_modes": []}

    def __init__(self, config: EnvConfig | None = None) -> None:
        if _IMPORT_ERROR is not None:
            raise RuntimeError(f"Missing dependency: {_IMPORT_ERROR}")
        if _MAVLINK_ERROR is not None:
            raise RuntimeError(f"Missing dependency: {_MAVLINK_ERROR}")

        self.config = config or EnvConfig()
        self.step_count = 0
        self.home: Optional[np.ndarray] = None
        self.targets: List[np.ndarray] = []
        self.target_index = 0
        self.last_step_time = 0.0

        obs_high = np.array([100, 100, 100, 10, 10, 10, 100, 100, 100], dtype=np.float32)
        act_high = np.array([self.config.max_speed] * 3 + [1.0], dtype=np.float32)
        self.observation_space = gym.spaces.Box(-obs_high, obs_high, dtype=np.float32)
        self.action_space = gym.spaces.Box(-act_high, act_high, dtype=np.float32)

        self.client = MavlinkClient(self.config.mavlink_url, self.config.action_rate_hz)

    def reset(
        self, *, seed: int | None = None, options: Dict[str, Any] | None = None
    ) -> Tuple[Any, Dict[str, Any]]:
        super().reset(seed=seed)
        self.step_count = 0
        self.target_index = 0
        self.home = None
        self.targets = []
        self.last_step_time = time.time()

        if self.config.auto_guided:
            self.client.set_mode_guided(self.config.guided_mode_id)
        if self.config.auto_arm:
            self.client.arm(self.config.arm_timeout)

        state = self.client.read_state(self.config.pose_timeout)
        if state is None:
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            return obs, {"pose_valid": False}

        pos, vel = state
        self.home = pos.copy()
        self.targets = self._build_targets(self.home)
        target = self.targets[0]
        obs = self._build_obs(pos, vel, target)
        return obs, {"pose_valid": True}

    def step(self, action: Any) -> Tuple[Any, float, bool, bool, Dict[str, Any]]:
        self.step_count += 1
        action = np.asarray(action, dtype=np.float32)
        action = np.clip(action, self.action_space.low, self.action_space.high)

        self._pace()
        self.client.send_velocity(action[0], action[1], action[2], action[3])

        state = self.client.read_state(self.config.pose_timeout)
        if state is None or not self.targets:
            obs = np.zeros(self.observation_space.shape, dtype=np.float32)
            return obs, -1.0, False, True, {"pose_valid": False}

        pos, vel = state
        target = self.targets[self.target_index]
        dist = float(np.linalg.norm(pos - target))
        reward = -dist

        terminated = False
        if dist <= self.config.target_radius:
            reward += 5.0
            self.target_index += 1
            if self.target_index >= len(self.targets):
                reward += 10.0
                terminated = True
                self.target_index = len(self.targets) - 1
            target = self.targets[self.target_index]

        obs = self._build_obs(pos, vel, target)
        truncated = self.step_count >= self.config.max_steps
        return obs, reward, terminated, truncated, {"pose_valid": True, "distance": dist}

    def close(self) -> None:
        try:
            self.client.send_velocity(0.0, 0.0, 0.0, 0.0)
            if self.config.disarm_on_close:
                self.client.disarm(self.config.arm_timeout)
        except Exception:
            pass

    def _build_targets(self, home: np.ndarray) -> List[np.ndarray]:
        targets = []
        for wp in self.config.waypoints:
            offset = np.array(wp, dtype=np.float32)
            targets.append(home + offset)
        return targets

    def _build_obs(self, pos: np.ndarray, vel: np.ndarray, target: np.ndarray) -> np.ndarray:
        return np.concatenate([pos, vel, target]).astype(np.float32)

    def _pace(self) -> None:
        if self.config.action_rate_hz <= 0:
            return
        interval = 1.0 / self.config.action_rate_hz
        now = time.time()
        elapsed = now - self.last_step_time
        if elapsed < interval:
            time.sleep(interval - elapsed)
        self.last_step_time = time.time()
