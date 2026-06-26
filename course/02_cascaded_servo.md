# 02 级联伺服控制：位置环、速度环、电流环

本章从单个 PID 位置控制过渡到工业伺服驱动器更常见的级联结构。

```text
位置环 -> 速度环 -> 电流环 -> 电压/PWM -> 电机
```

## 学习目标

学完本章后，你应该能判断：

- 为什么位置控制不宜直接跳到 PWM，而要分成多层闭环。
- 为什么电流环通常最快，速度环其次，位置环最慢。
- 为什么每层环都需要限幅，限幅会如何影响上层控制器。
- 为什么机械臂轨迹的速度、加速度和力矩限制最终会落到这些环路上。

## 为什么要级联

单 PID 位置控制把位置误差直接变成电压，结构简单，但工程上有几个问题：

- 电流没有被直接控制，等价于力矩没有被明确限制。
- 速度响应完全由位置环间接决定，调参耦合严重。
- 电压饱和后，位置环积分可能继续累积。
- 很难把动力学前馈、速度前馈、力矩前馈干净地加进去。

级联控制把问题拆开：

```text
位置误差 -> 期望速度
速度误差 -> 期望电流或力矩
电流误差 -> 期望电压
```

这和机械臂控制接口更匹配。规划器或动力学控制器可以给位置、速度、力矩前馈，底层伺服负责把这些指令落到电机侧。

## 带宽关系

经验上，内环必须比外环快：

```text
电流环带宽 > 速度环带宽 > 位置环带宽
```

如果位置环太快，而速度环或电流环跟不上，会出现：

- 位置环持续要求更高速度。
- 速度环持续要求更大电流。
- 电流或电压碰到限幅。
- 外环继续积累误差，解除限幅后过冲或振荡。

在真实机械臂里，这类问题常表现为轨迹拐点附近抖动、跟踪误差尖峰、关节发热、驱动器过流报警。

## 本章代码结构

控制器代码在 [include/motor_control/control/CascadedServoController.hpp](../include/motor_control/control/CascadedServoController.hpp)。

核心接口是：

```text
CascadedServoCommand:
  targetPosition
  velocityFeedforward
  currentFeedforward

CascadedServoFeedback:
  position
  velocity
  current

CascadedServoOutput:
  velocitySetpoint
  currentSetpoint
  voltageCommand
```

示例程序在 [examples/sim_cascaded_servo.cpp](../examples/sim_cascaded_servo.cpp)。它使用平滑阶跃作为目标位置，并在中途加入负载扰动。

## 运行实验

```bash
cmake --build build
./build/examples/sim_cascaded_servo
```

输出文件：

```text
outputs/cascaded_servo.csv
```

建议画这些列：

- `target_position_rad` 和 `position_rad`：看外环跟踪。
- `velocity_setpoint_rad_s` 和 `velocity_rad_s`：看速度环是否跟上。
- `current_setpoint_a` 和 `current_a`：看电流环是否跟上。
- `voltage_command_v`：看电压是否长期饱和。
- `load_torque_nm`：看扰动进入后的恢复过程。

## 和机械臂算法接口的关系

规控算法常输出：

```text
q_des, dq_des, ddq_des
```

动力学控制可能进一步输出：

```text
tau_des = M(q) ddq_des + C(q, dq) dq + g(q) + tau_feedback
```

对伺服驱动器来说，`tau_des` 需要通过减速比和电机力矩常数换算成电流前馈。这个学习包的 `currentFeedforward` 就是为后续扩展这个接口预留的。

## 实验建议

1. 把 `maxCurrent` 从 5 A 改成 2 A，观察位置跟踪是否明显变差。
2. 把 `maxVelocity` 从 6 rad/s 改成 3 rad/s，观察位置轨迹是否被速度限制截断。
3. 把速度环 `ki` 改成 0，观察负载扰动后的恢复能力。
4. 把位置目标从平滑阶跃改成硬阶跃，观察电流和电压峰值。
5. 把 `dt` 改成 0.002 或 0.005，观察同一组参数是否仍然稳定。

## 常见坑

- 只给位置目标，不给速度或力矩前馈，会让反馈环承担全部动态需求。
- 外环带宽设置过高时，内环实际成为瓶颈，表面看像“位置环不好调”。
- 电流限幅太低时，关节会显得“软”；电流限幅太高时，可能带来发热和安全风险。
- 规划轨迹不平滑时，速度环和电流环会看到尖峰需求，即使位置曲线本身看起来连续。
