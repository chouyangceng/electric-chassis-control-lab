from electric_chassis_control.experiments import run

if __name__ == "__main__":
    result = run(steps=200)
    print(result.metrics)
