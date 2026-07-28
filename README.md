# Electric Chassis Control Lab

面向车辆工程本科生的四轮独立驱动车辆动力学、横摆稳定控制和扭矩分配实验平台。项目重点是 7 自由度车辆模型、联合滑移轮胎约束、DYC/LQR/NMPC 基线和可解释的四轮扭矩分配。

## 30 秒运行

    python -m pip install -e .
    python examples/quickstart.py

## 完整实验

    python experiments/run_experiment.py --scenario double_lane_change --controller dyc
    python experiments/run_experiment.py --scenario split_mu --controller nmpc
    python -m pytest -q
    python -m ruff check .

结果包括 `metrics.json`、`trace.csv` 和控制曲线。实验支持 `double_lane_change`、`split_mu`、`low_friction` 和 `motor_degradation`。

## 研究问题

- 轮胎纵横向联合滑移为什么要受摩擦圆约束？
- 直接横摆力矩、LQR 和 NMPC 在低附着工况下有什么差异？
- 四轮独立扭矩如何同时满足纵向力、横摆力矩和执行器边界？
- 执行器受限或单电机降级时如何保持可控？

## 与轨迹跟踪项目的区别

本项目不以路径跟踪为主要问题，而以底盘稳定性、横摆力矩、扭矩分配、制动和降级为重点。

## CARLA/ROS 2 边界

CARLA/ROS 2 适配器可用于验证高层转向、油门和制动接口；四轮独立扭矩控制在自研 7 自由度模型中验证，因为标准 CARLA 车辆控制接口不等于真实四轮电机控制器。项目没有实车实验，所有数字均为仿真结果。

### ROS 2 可选接口

仓库现在包含一个不侵入核心依赖的 ROS 2 Python 包：
`ros2/electric_chassis_control_ros`。它将 `geometry_msgs/msg/Twist` 转换为四轮
扭矩/制动命令，并通过 `std_msgs/msg/Float64MultiArray` 和
`diagnostic_msgs/msg/DiagnosticArray` 发布结果。桥接层在发布前再次执行扭矩和制动限幅，
可在没有 `rclpy` 的普通 Python 环境中运行测试。完整消息约定、参数和 `colcon` 构建步骤见
[docs/ros2接口.md](docs/ros2接口.md)。

## License

Apache-2.0

## 高级控制基准

    python examples/advanced_benchmark.py

高级示例会统一比较无控制、DYC、LQR 和 NMPC，并额外计算 ABS/再生制动协调和不同附着系数下的稳定性边界。新增模块包括 `controllers/esc.py`、`controllers/abs.py`、`allocation/constrained.py`、`allocation/energy.py`、`simulation/envelope.py` 和 `metrics/benchmark.py`。
