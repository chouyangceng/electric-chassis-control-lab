# Mathematical Model

状态向量为 `[vx, vy, yaw_rate, wheel_speed_fl, wheel_speed_fr, wheel_speed_rl, wheel_speed_rr]`。车轮纵向滑移由轮速和车速计算，侧偏角由车身速度与横摆率计算。轮胎纵向力和侧向力先按刚度模型计算，再投影到摩擦圆内。

扭矩分配器将目标纵向力和附加横摆力矩转换为四个轮端扭矩，并执行电机上限检查。控制器输入和输出均做有限值与维度校验。
