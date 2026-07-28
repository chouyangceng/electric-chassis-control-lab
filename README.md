# Electric Chassis Control Lab

面向车辆工程本科生的四轮独立驱动车辆动力学、横摆稳定控制和扭矩分配实验平台。项目以代码和
可复现实验为主，包含 7 自由度车辆模型、联合滑移轮胎约束、DYC/LQR/NMPC 控制基线、
ESC/ABS、再生制动协调和四轮扭矩分配。

> 本仓库当前结果来自软件仿真，尚未完成实车验证。ROS 2 接口用于规范连接仿真器、台架或后续
> 硬件控制器，不能替代真实车辆的功能安全设计与认证。

## 快速运行

```bash
python -m pip install -e .
python examples/quickstart.py
python examples/advanced_benchmark.py
```

## 完整实验

```bash
python experiments/run_experiment.py --scenario double_lane_change --controller dyc
python experiments/run_experiment.py --scenario split_mu --controller nmpc
python -m pytest -q
python -m ruff check .
```

实验支持 `double_lane_change`、`split_mu`、`low_friction` 和 `motor_degradation` 工况，
输出 `metrics.json`、`trace.csv` 和控制曲线。高级基准统一比较无控制、DYC、LQR 与 NMPC，
并计算 ABS、再生制动协调和不同附着系数下的稳定性边界。

## ROS 2 安全接口

可选 ROS 2 Python 包位于 `ros2/electric_chassis_control_ros`，不会把 `rclpy` 加入核心算法
依赖。节点采用语义明确的标准消息：

- `geometry_msgs/msg/Wrench` 表达纵向合力和横摆力矩；
- 两个 `std_msgs/msg/Float64` 分别表达转角和制动请求；
- `Float64MultiArray` 发布四轮扭矩和四轮制动压力；
- 输入非法或超时后，看门狗发布零驱动扭矩和可配置的安全制动；
- 分配残差、饱和、限幅和故障安全状态发布到 `/diagnostics`；
- 四轮轮速使用独立消息，不占用 `Odometry` 的 covariance 字段。

完整主题、参数、构建步骤和安全行为见 [ROS 2 接口说明](docs/ros2接口.md)。

## 研究问题

- 轮胎纵横向联合滑移为什么受到摩擦圆约束？
- 直接横摆力矩、LQR 和 NMPC 在低附着工况下有何差异？
- 四轮独立扭矩如何同时满足纵向力、横摆力矩和执行器边界？
- 执行器受限或单电机降级时，如何维持车辆稳定性与可控性？
- 命令链路延迟或故障时，底盘控制器如何进入确定性的安全状态？

## 项目边界

本项目不以路径跟踪为主要问题，而以底盘稳定性、横摆力矩、扭矩分配、制动与降级控制为重点。
标准 CARLA 车辆接口不等同于真实四轮电机控制器，因此四轮独立扭矩控制主要在自研 7 自由度
模型中验证。

## License

Apache-2.0
