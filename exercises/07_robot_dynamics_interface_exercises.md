# 07 机械臂动力学接口练习

对应教材：[course/07_robot_dynamics_interface.md](../course/07_robot_dynamics_interface.md)

## 实验 1：computed torque 轨迹跟踪

运行：

```bash
./build/examples/sim_two_link_computed_torque
```

画出：

- `q1_des_rad` 和 `q1_rad`
- `q2_des_rad` 和 `q2_rad`
- `q1_error_rad` 和 `q2_error_rad`
- `tau1_nm` 和 `tau2_nm`

回答：

1. 最大误差出现在轨迹哪个阶段？
2. 力矩峰值和加速度阶段是否相关？
3. 两个关节的误差是否互相影响？

## 实验 2：反馈增益

修改 [examples/sim_two_link_computed_torque.cpp](../examples/sim_two_link_computed_torque.cpp) 中的 `kp` 和 `kd`。

回答：

1. 增大 `kp` 后误差是否减小？
2. 增大 `kp` 是否提高力矩峰值？
3. `kd` 太小时是否更容易振荡？

## 实验 3：力矩限幅

修改 [include/motor_control/robot/TwoLinkArm.hpp](../include/motor_control/robot/TwoLinkArm.hpp) 中默认 `torqueLimit`，从 25 Nm 改成 10 Nm。

回答：

1. 跟踪误差是否明显变大？
2. 哪个关节更容易受限？
3. 规划层可以如何降低力矩需求？

## 实验 4：轨迹时长

把轨迹结束时间从 `2.2/2.4` 秒缩短到 `1.2/1.4` 秒。

回答：

1. 更快轨迹是否导致更大力矩峰值？
2. 跟踪误差是否增大？
3. 为什么轨迹平滑仍然不等于动力学可实现？
