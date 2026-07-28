def test_controller_benchmark_contains_all_baselines():
    from electric_chassis_control.metrics.benchmark import benchmark_controllers

    result = benchmark_controllers(steps=80, seed=5)
    assert set(result) == {"none", "dyc", "lqr", "nmpc"}
    assert all("yaw_rate_rmse" in metrics for metrics in result.values())
