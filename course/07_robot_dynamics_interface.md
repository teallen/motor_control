# 07 机械臂动力学与控制接口

本章把规划轨迹连接到关节力矩控制。规划算法输出 `q(t), dq(t), ddq(t)`，如果只把 `q` 交给位置环，动态需求会全部落到反馈控制上。更好的方式是把动力学前馈也传给底层。

## 学习目标

学完本章后，你应该能判断：

- 规划轨迹如何变成期望关节力矩。
- computed torque control 为什么能降低跟踪误差。
- 力矩限幅为什么会破坏理想动力学补偿。
- jerk、加速度和力矩连续性为什么会影响驱动器可实现性。

## 从轨迹到力矩

机械臂动力学常写成：

```text
tau = M(q) ddq + C(q, dq) dq + g(q)
```

computed torque 在此基础上加入反馈项：

```text
ddq_cmd = ddq_des + Kd * (dq_des - dq) + Kp * (q_des - q)
tau_cmd = M(q) ddq_cmd + C(q, dq) dq + g(q)
```

这样，前馈负责主要动态需求，反馈只修正模型误差和扰动。

## 2-DOF 平面臂模型

本学习包的双连杆模型在 [include/motor_control/robot/TwoLinkArm.hpp](../include/motor_control/robot/TwoLinkArm.hpp)。它包含：

- 质量矩阵 `M(q)`。
- 科氏/离心项。
- 重力项。
- 关节阻尼。
- 力矩限幅。

示例 [examples/sim_two_link_computed_torque.cpp](../examples/sim_two_link_computed_torque.cpp) 让两个关节跟踪平滑轨迹，并输出目标、实际位置、误差和力矩。

## 力矩限幅

真实机械臂有关节连续力矩、峰值力矩、电机电流、母线电压和热限制。即使 computed torque 公式正确，如果 `tau_cmd` 超过限制：

```text
tau_cmd 被截断 -> 实际 ddq 不足 -> 轨迹误差增大 -> 反馈项继续增大 -> 可能进一步饱和
```

所以规划层应该尽量生成动力学可实现的轨迹，而不是把不可实现的轨迹交给底层“硬跟”。

## 轨迹平滑性

位置连续不代表力矩连续。力矩主要和加速度、速度耦合和重力有关：

- 速度不连续：需要无限加速度。
- 加速度不连续：力矩阶跃，容易激发结构振动。
- jerk 太大：驱动器电流变化快，轨迹拐点可能出现误差尖峰。

规划器需要关注 `q, dq, ddq`，而不仅是几何路径。

## 和电机控制的接口

关节力矩最终会换算到电机侧：

```text
tau_motor = tau_joint / gear_ratio
Iq_ref = tau_motor / Kt
```

如果底层只提供位置模式，你的 computed torque 前馈可能无法直接进入驱动器；如果提供力矩模式，也要确认电流环带宽、延迟和限幅是否满足任务需求。

## 运行实验

```bash
cmake --build build
./build/examples/sim_two_link_computed_torque
```

输出文件：

```text
outputs/two_link_computed_torque.csv
```

建议画：

- `q1_des_rad` 和 `q1_rad`
- `q2_des_rad` 和 `q2_rad`
- `q1_error_rad` 和 `q2_error_rad`
- `tau1_nm` 和 `tau2_nm`

观察力矩峰值和误差尖峰是否出现在轨迹加速度较大的阶段。
