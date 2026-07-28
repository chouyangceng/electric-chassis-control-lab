from __future__ import annotations

from pathlib import Path

from ..experiments.runner import run


def benchmark_controllers(steps: int = 400, seed: int = 7, output_root: str | Path = "artifacts/benchmark") -> dict[str, dict[str, float]]:
    if steps < 2:
        raise ValueError("steps must be at least 2")
    root = Path(output_root)
    result = {}
    for controller in ("none", "dyc", "lqr", "nmpc"):
        metrics = run("double_lane_change", steps, root / controller, controller).metrics
        result[controller] = {key: value for key, value in metrics.items() if isinstance(value, float)}
    return result
