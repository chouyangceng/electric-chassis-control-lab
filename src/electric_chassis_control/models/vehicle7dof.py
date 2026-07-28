from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .tire import combined_tire_force


@dataclass(frozen=True)
class Vehicle7DOF:
    mass: float = 1600.0
    yaw_inertia: float = 2800.0
    front_axle: float = 1.25
    rear_axle: float = 1.55
    track: float = 1.58
    wheel_radius: float = 0.32
    wheel_inertia: float = 2.0
    max_torque: float = 2500.0
    friction: float = 0.85
    gravity: float = 9.81

    @classmethod
    def default(cls) -> Vehicle7DOF:
        return cls()

    def __post_init__(self) -> None:
        values = [self.mass, self.yaw_inertia, self.front_axle, self.rear_axle, self.track,
                  self.wheel_radius, self.wheel_inertia, self.max_torque]
        if any(value <= 0 for value in values):
            raise ValueError("vehicle parameters must be positive")

    def clip_torques(self, torques: np.ndarray) -> np.ndarray:
        torques = np.asarray(torques, dtype=float)
        if torques.shape != (4,) or not np.all(np.isfinite(torques)):
            raise ValueError("torques must be a finite vector with shape (4,)")
        return np.clip(torques, -self.max_torque, self.max_torque)

    def tire_force(self, slip_ratio: float, slip_angle: float, normal_load: float,
                   friction: float | None = None) -> tuple[float, float]:
        return combined_tire_force(slip_ratio, slip_angle, normal_load, friction or self.friction)

    def derivative(self, state: np.ndarray, steering: float, torques: np.ndarray,
                   friction: float | None = None) -> np.ndarray:
        state = np.asarray(state, dtype=float)
        torques = self.clip_torques(torques)
        if state.shape != (7,) or not np.all(np.isfinite(state)):
            raise ValueError("state must be a finite vector with shape (7,)")
        vx, vy, yaw_rate, *wheel_speeds = state
        vx_safe = max(abs(vx), 0.5)
        loads = np.array([0.26, 0.26, 0.24, 0.24]) * self.mass * self.gravity
        angles = np.array([
            steering - (vy + self.front_axle * yaw_rate) / vx_safe,
            steering - (vy + self.front_axle * yaw_rate) / vx_safe,
            -(vy - self.rear_axle * yaw_rate) / vx_safe,
            -(vy - self.rear_axle * yaw_rate) / vx_safe,
        ])
        slips = (self.wheel_radius * np.asarray(wheel_speeds) - vx) / vx_safe
        forces = np.array([self.tire_force(s, a, n, friction) for s, a, n in zip(slips, angles, loads)])
        fx = forces[:, 0].sum()
        fy = forces[0, 1] * np.cos(steering) + forces[1, 1] * np.cos(steering) + forces[2:, 1].sum()
        moment = self.front_axle * (forces[0, 1] + forces[1, 1]) * np.cos(steering)
        moment -= self.rear_axle * (forces[2, 1] + forces[3, 1])
        dvx = (fx - self.mass * vy * yaw_rate) / self.mass
        dvy = (fy + self.mass * vx * yaw_rate) / self.mass
        dr = moment / self.yaw_inertia
        dw = (torques - self.wheel_radius * forces[:, 0]) / self.wheel_inertia
        return np.array([dvx, dvy, dr, *dw], dtype=float)

    def step(self, state: np.ndarray, steering: float, torques: np.ndarray,
             dt: float = 0.01, friction: float | None = None) -> np.ndarray:
        if dt <= 0:
            raise ValueError("dt must be positive")
        return np.asarray(state, dtype=float) + dt * self.derivative(state, steering, torques, friction)
