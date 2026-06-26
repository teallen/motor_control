# 01 电机模型与 PID 位置控制

本章目标是用一个最小 DC 电机模型理解电压、电流、速度、位置和力矩之间的关系，并用 PID 做位置闭环。

## 学习目标

学完本章后，你应该能判断：

- 为什么“电流近似等于力矩”是机器人关节控制里非常重要的近似。
- 为什么位置环增益调大后，响应变快但更容易超调、振荡或撞限幅。
- 为什么负载扰动会造成位置误差，以及积分项如何消除稳态误差。
- 为什么真实驱动器中的电压、电流限制会改变你在算法仿真中看到的理想响应。

## DC 电机模型

简化 DC 电机可以分成电气方程和机械方程：

```text
电气方程
V = R i + L di/dt + Ke w
机械方程
J dw/dt = Kt i - tau_load - b w - tau_friction
运动学关系
dtheta/dt = w
```


其中：

- `V`：电机端电压。
- `i`：电枢电流。
- `w`：电机角速度。
- `theta`：电机角位置。
- `R, L`：电阻和电感。
- `Kt`：力矩常数，决定电流到力矩的比例。
- `Ke`：反电动势常数，速度越高，需要越多电压抵消反电动势。
- `J, b`：转动惯量和粘性摩擦。

代码对应 [include/motor_control/motor/DcMotor.hpp](../include/motor_control/motor/DcMotor.hpp)。

## PID 位置控制

最简单的位置控制是：

```text
error = target_position - measured_position
voltage = Kp * error + Ki * integral(error) + Kd * derivative(error)
```

在这个学习包里，PID 直接输出电压。这不是工业伺服驱动器的最终结构，但很适合建立第一层直觉：

- `Kp` 提供主要刚度，越大越像“弹簧”。
- `Ki` 消除稳态误差，但容易积分饱和。
- `Kd` 提供阻尼，能降低超调，但会放大测量噪声。

代码对应 [include/motor_control/control/PidController.hpp](../include/motor_control/control/PidController.hpp)。

## 和机械臂规划的关系

机械臂轨迹规划通常关心位置、速度、加速度和 jerk 连续性。对底层电机来说，过激的轨迹会表现为：

- 位置误差瞬间变大，位置环要求更高速度或电压。
- 加速度要求变大，需要更大的电流和力矩。
- 速度升高后反电动势增大，同样母线电压下可用控制裕度变小。
- 控制输出饱和后，积分项继续积累，解除饱和时可能导致过冲。

所以，规划层的“轨迹能不能跟”最终会落到电机侧的电流、电压、带宽和热限制上。

## 运行实验

```bash
cmake -S . -B build
cmake --build build
./build/examples/sim_dc_position_pid
```

输出文件：

```text
outputs/dc_position_pid.csv
```

建议画这些列：

- `target_position_rad` 和 `position_rad`：看位置跟踪。
- `velocity_rad_s`：看速度峰值和振荡。
- `current_a` 和 `torque_nm`：看力矩需求。
- `voltage_v`：看是否长期贴近 ±24 V。
- `load_torque_nm`：看负载扰动进入后的恢复过程。

## 实验建议

1. 把 `kp` 从 28 改到 10，再改到 60，观察响应速度和超调。
2. 把 `ki` 改成 0，观察负载扰动后的稳态误差。
3. 把 `maxOutput` 从 ±24 V 改到 ±12 V，观察同样目标下是否更慢或更难恢复。
4. 把 `rotorInertia` 放大 5 倍，类比机械臂负载变重后的跟踪变化。

## 常见坑

- 只看位置误差，不看电流和电压，会误判控制器是否可在真实驱动器上落地。
- 积分项没有限幅时，饱和阶段可能积累过多误差，导致解除饱和后大幅过冲。
- 采样周期变大后，同样 PID 参数可能从稳定变成振荡。
- 电机高速时反电动势吃掉电压裕度，低速下能输出的力矩不代表高速下也能输出。
