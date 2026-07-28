from __future__ import annotations

import argparse

from electric_chassis_control.experiments import run


def main() -> None:
    parser = argparse.ArgumentParser(description="Run an electric chassis control scenario")
    parser.add_argument("--scenario", default="double_lane_change")
    parser.add_argument("--controller", default="dyc", choices=["none", "dyc", "lqr", "nmpc"])
    parser.add_argument("--steps", type=int, default=400)
    parser.add_argument("--output", default="artifacts/chassis-control")
    args = parser.parse_args()
    result = run(args.scenario, args.steps, args.output, args.controller)
    print(result.metrics)


if __name__ == "__main__":
    main()
