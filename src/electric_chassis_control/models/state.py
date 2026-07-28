from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ChassisState:
    vx: float
    vy: float
    yaw_rate: float
    sideslip: float
    wheel_speeds: np.ndarray

    def __post_init__(self) -> None:
        wheels = np.asarray(self.wheel_speeds, dtype=float)
        if wheels.shape != (4,) or not np.all(np.isfinite(wheels)):
            raise ValueError("wheel_speeds must be a finite vector with shape (4,)")
        if self.vx < 0 or not np.all(np.isfinite([self.vx, self.vy, self.yaw_rate, self.sideslip])):
            raise ValueError("chassis state contains invalid speed or angle values")


@dataclass(frozen=True)
class ChassisCommand:
    steering: float
    wheel_torques: np.ndarray
    brake_pressures: np.ndarray
    diagnostics: dict[str, float]

    def __post_init__(self) -> None:
        torques = np.asarray(self.wheel_torques, dtype=float)
        brakes = np.asarray(self.brake_pressures, dtype=float)
        if torques.shape != (4,) or brakes.shape != (4,):
            raise ValueError("wheel commands must have shape (4,)")
        if not np.all(np.isfinite(torques)) or not np.all(np.isfinite(brakes)):
            raise ValueError("wheel commands must be finite")
