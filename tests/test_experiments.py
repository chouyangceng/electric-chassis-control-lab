from pathlib import Path


def test_double_lane_change_metrics():
    from electric_chassis_control.experiments import run

    result = run(scenario="double_lane_change", steps=100, output_dir=Path("artifacts/test-control"))
    assert {"yaw_rate_rmse", "sideslip_peak", "torque_saturation"} <= set(result.metrics)
