# ROS 2 接口说明

ROS 2 适配包位于 `ros2/electric_chassis_control_ros`。核心动力学、控制器和分配器不依赖
`rclpy`，因此普通 Python 环境仍可运行算法、实验与单元测试；只有部署节点时才需要 ROS 2。

## 控制输入

节点不再使用 `geometry_msgs/msg/Twist` 冒充车辆控制命令。控制输入拆分为三个语义明确的
标准消息：

| 主题 | 消息类型 | 字段及单位 |
| --- | --- | --- |
| `~/force_request` | `geometry_msgs/msg/Wrench` | `force.x`：纵向合力 N；`torque.z`：横摆力矩 N·m |
| `~/steering_angle` | `std_msgs/msg/Float64` | 前轮转角 rad |
| `~/brake_request` | `std_msgs/msg/Float64` | 归一化制动压力 0～1 |

只有三类输入均已收到且时间差不超过 `command_timeout_s` 时，节点才接受新命令。节点输出：

- `~/wheel_torques`：四轮扭矩，`Float64MultiArray`，顺序为前左、前右、后左、后右；
- `~/brake_pressures`：四轮归一化制动压力，顺序相同；
- `~/steering_command`：实际下发的前轮转角，`Float64`，单位 rad；
- `/diagnostics`：限幅、分配残差、分配饱和与故障安全状态。

## 看门狗与故障安全

`CommandWatchdog` 是不依赖 ROS 2 的纯 Python 安全边界。输入超时、包含 NaN/Inf 或消息格式
非法时，节点立即或在下一看门狗周期发布确定性的安全输出：

- 四轮驱动扭矩全部为 0；
- 转向请求归零；
- 四轮制动压力设为 `safe_brake_pressure`；
- `/diagnostics` 发布 ERROR，并说明超时或非法输入原因。

非法输入会清空三类输入缓存。故障安全锁存只有在 Wrench、转角、制动三类输入全部重新到达
且时间差满足阈值后才会解除，单个新消息不会与拒绝前的缓存拼接成命令。

制动参数受到双重约束：`max_brake_pressure` 必须位于 `(0, 1]`，且
`safe_brake_pressure <= max_brake_pressure`。`allocator_residual_warn_threshold` 用于判定
扭矩分配残差或饱和，超过阈值时诊断等级为 WARN。

## 状态消息约定

`Ros2CommandBridge.state_to_messages()` 使用标准 `nav_msgs/msg/Odometry` 字段表达纵向速度、
侧向速度和横摆角速度。轮速不会写入 covariance：

- 四轮轮速使用单独的 `Float64MultiArray`；
- 质心侧偏角使用单独的 `Float64`；
- Odometry covariance 保留给真实的估计协方差。

Odometry 默认使用 `header.frame_id=odom` 和 `child_frame_id=base_link`，避免把父子坐标系写成
同一个值；调用桥接函数时可显式传入非负秒时间戳，安全填充 `stamp.sec/nanosec`。

## 参数

默认参数位于 `config/controller.yaml`：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `max_torque` | 2500.0 | 单轮最大绝对扭矩 N·m |
| `max_brake_pressure` | 1.0 | 归一化最大制动压力 |
| `safe_brake_pressure` | 0.7 | 故障安全制动压力 |
| `command_timeout_s` | 0.25 | 命令超时阈值 s |
| `watchdog_period_s` | 0.05 | 安全检查周期 s |
| `allocator_residual_warn_threshold` | 1.0 | 分配残差告警阈值 |

启动文件会从安装后的包共享目录加载这份 YAML，而不是在 Python 中重复硬编码参数。

## 构建与运行

```bash
python -m pip install -e .
cd ~/ros2_ws/src
ln -s /path/to/electric-chassis-control-lab/ros2/electric_chassis_control_ros .
cd ~/ros2_ws
colcon build --symlink-install --packages-select electric_chassis_control_ros
source install/setup.bash
ros2 launch electric_chassis_control_ros controller.launch.py
```

可用下面的命令发送一组完整输入：

```bash
ros2 topic pub --once /electric_chassis_controller/force_request geometry_msgs/msg/Wrench \
  "{force: {x: 1200.0}, torque: {z: 300.0}}"
ros2 topic pub --once /electric_chassis_controller/steering_angle std_msgs/msg/Float64 "{data: 0.05}"
ros2 topic pub --once /electric_chassis_controller/brake_request std_msgs/msg/Float64 "{data: 0.0}"
```

没有 ROS 2 时，`python -m pytest -q` 仍会测试消息转换、物理限幅、分配告警和看门狗逻辑。
