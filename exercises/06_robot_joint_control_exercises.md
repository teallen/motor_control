# 06 机器人单关节控制练习

对应教材：[course/06_robot_joint_control.md](../course/06_robot_joint_control.md)

## 实验 1：重力和摩擦补偿

运行：

```bash
./build/examples/sim_single_joint_gravity_friction
```

画出：

- `target_position_rad`
- `position_feedback_only_rad`
- `position_compensated_rad`
- `error_feedback_only_rad`
- `error_compensated_rad`

回答：

1. 加入补偿后跟踪误差是否降低？
2. 补偿主要改善轨迹哪个阶段？
3. 如果重力模型参数不准，可能出现什么问题？

## 实验 2：摩擦参数

修改 [include/motor_control/robot/SingleJoint.hpp](../include/motor_control/robot/SingleJoint.hpp) 中默认 `coulombFriction`。

回答：

1. 库仑摩擦变大后，低速跟踪误差如何变化？
2. 摩擦前馈如果符号错了，会发生什么？
3. 机械臂实际调试中如何辨识摩擦？

## 实验 3：阻抗刚度

运行：

```bash
./build/examples/sim_joint_impedance
```

画出：

- `external_torque_nm`
- `soft_position_rad`
- `stiff_position_rad`
- `soft_torque_nm`
- `stiff_torque_nm`

回答：

1. 软阻抗在外力下位移是否更大？
2. 硬阻抗是否需要更大的控制力矩？
3. 末端接触任务为什么不能只追求高刚度？

## 实验 4：阻尼

把硬阻抗控制器的 `damping` 从 8 改成 2 和 15。

回答：

1. 阻尼太低是否更容易振荡？
2. 阻尼太高是否让响应变慢？
3. 阻尼项为什么对速度估计噪声敏感？
