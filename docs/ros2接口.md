# ROS 2 接口

项目的 ROS 2 适配器位于 `ros2/electric_chassis_control_ros`，不会把 `rclpy`
加入核心 Python 依赖。这样可以在普通 Python/CI 环境运行全部动力学和控制算法，
在 ROS 2 Jazzy/Humble 工作区中再构建节点。

## 消息约定

节点订阅 `~/command`（`geometry_msgs/msg/Twist`）：

| 字段 | 含义 | 单位 |
| --- | --- | --- |
| `linear.x` | 前轮转角 | rad |
| `linear.y` | 纵向合力请求 | N |
| `linear.z` | 四轮统一制动压力 | 0–1 |
| `angular.z` | 横摆力矩请求 | N·m |

节点发布 `~/wheel_torques` 和 `~/brake_pressures`（`std_msgs/msg/Float64MultiArray`），
数组顺序为前左、前右、后左、后右；同时发布 `/diagnostics`
（`diagnostic_msgs/msg/DiagnosticArray`）。所有输出在发布前都会再次执行扭矩和制动限幅。

## 构建与运行

先安装核心项目，再把 ROS 包放入 ROS 2 工作区：

```bash
pip install -e .
cd ~/ros2_ws/src
ln -s /path/to/electric-chassis-control-lab/ros2/electric_chassis_control_ros .
cd ~/ros2_ws
colcon build --symlink-install --packages-select electric_chassis_control_ros
source install/setup.bash
ros2 launch electric_chassis_control_ros controller.launch.py
```

没有 ROS 2 时，`python -m pytest -q` 仍会运行 bridge 的安全边界测试；导入
`Ros2CommandBridge` 不需要 `rclpy`。
