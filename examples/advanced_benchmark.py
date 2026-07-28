from electric_chassis_control.allocation import RegenerativeBrakeCoordinator
from electric_chassis_control.metrics import benchmark_controllers
from electric_chassis_control.simulation import compute_stability_envelope

if __name__ == "__main__":
    comparison = benchmark_controllers(steps=300, seed=12)
    regen = RegenerativeBrakeCoordinator().allocate(3000.0, battery_soc=0.55)
    envelope = compute_stability_envelope([0.2, 0.5, 0.8], speed=15.0)
    print({"controllers": comparison, "regen": regen, "stability_envelope": envelope.tolist()})
