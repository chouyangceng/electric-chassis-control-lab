from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from ..allocation.qp_allocator import TorqueAllocator
from ..controllers.dyc import DirectYawMomentController
from ..controllers.lqr import LQRController
from ..controllers.nmpc import NMPCController


@dataclass(frozen=True)
class ExperimentResult:
    metrics: dict[str, float]
    output_dir: Path


def run(scenario: str = "double_lane_change", steps: int = 400,
        output_dir: str | Path = "artifacts/chassis-control", controller: str = "dyc") -> ExperimentResult:
    if scenario not in {"double_lane_change", "split_mu", "low_friction", "motor_degradation"}:
        raise ValueError("unknown chassis scenario")
    if steps < 2:
        raise ValueError("steps must be at least 2")
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    dt = 0.01
    t = np.arange(steps) * dt
    target = 0.18 * np.sin(0.8 * t) if scenario == "double_lane_change" else 0.12 * np.sin(0.5 * t)
    yaw = 0.0
    sideslip = 0.0
    allocator = TorqueAllocator.default()
    dyc = DirectYawMomentController()
    lqr = LQRController()
    nmpc = NMPCController()
    yaw_trace = np.zeros(steps)
    sideslip_trace = np.zeros(steps)
    torque_trace = np.zeros((steps, 4))
    for i in range(steps):
        if controller == "none":
            moment = 0.0
        elif controller == "lqr":
            moment = lqr.compute(np.array([target[i] - yaw, -sideslip]))
        elif controller == "nmpc":
            moment = nmpc.compute(target[i] - yaw, sideslip, dt)
        else:
            moment = float(dyc.compute(15.0, yaw, target[i], sideslip)[0])
        friction = 0.45 if scenario in {"split_mu", "low_friction"} and i > steps // 2 else 0.85
        force = 800.0 * (1.0 if scenario != "motor_degradation" else 0.7)
        torques = allocator.allocate(force, moment)
        yaw_rate = 0.65 * (target[i] - yaw) + 0.00025 * moment
        yaw += dt * yaw_rate
        sideslip += dt * (-2.5 * sideslip + 0.18 * yaw_rate / max(friction, 0.1))
        yaw_trace[i] = yaw
        sideslip_trace[i] = sideslip
        torque_trace[i] = torques
    metrics = {
        "scenario": scenario,
        "controller": controller,
        "yaw_rate_rmse": float(np.sqrt(np.mean((yaw_trace - target) ** 2))),
        "sideslip_peak": float(np.max(np.abs(sideslip_trace))),
        "torque_saturation": float(np.mean(np.abs(torque_trace) >= allocator.max_torque - 1e-8)),
    }
    np.savetxt(output / "trace.csv", np.column_stack([t, target, yaw_trace, sideslip_trace, torque_trace]),
               delimiter=",", header="time,target_yaw,yaw,sideslip,tfl,tfr,trl,trr", comments="")
    (output / "metrics.json").write_text(json.dumps(metrics, indent=2), encoding="utf-8")
    try:
        import matplotlib.pyplot as plt

        figure, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        axes[0].plot(t, target, label="target yaw")
        axes[0].plot(t, yaw_trace, label="yaw")
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[1].plot(t, sideslip_trace, label="sideslip")
        axes[1].set_xlabel("time (s)")
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        figure.tight_layout()
        figure.savefig(output / "control_result.png", dpi=140)
        plt.close(figure)
    except ImportError:
        pass
    return ExperimentResult(metrics, output)
