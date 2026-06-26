# 02 级联伺服控制练习

对应教材：[course/02_cascaded_servo.md](../course/02_cascaded_servo.md)

## 实验 1：观察三层环路

运行：

```bash
./build/examples/sim_cascaded_servo
```

画出：

- `target_position_rad` 和 `position_rad`
- `velocity_setpoint_rad_s` 和 `velocity_rad_s`
- `current_setpoint_a` 和 `current_a`
- `voltage_command_v`

回答：

1. 速度环是否明显滞后于速度设定？
2. 电流环是否能快速跟上电流设定？
3. 负载扰动进入后，哪一个变量最先发生变化？

## 实验 2：电流限幅

把 `maxCurrent` 从 5 A 改成 2 A。

回答：

1. 位置误差是否变大？
2. 速度设定和实际速度之间的差距是否变大？
3. 如果这是机器人关节，用户会感受到“更软”还是“更硬”？

## 实验 3：速度限幅

把 `maxVelocity` 从 6 rad/s 改成 3 rad/s。

回答：

1. 目标轨迹是否仍能按原速度跟踪？
2. 电流峰值是否降低？
3. 这和规划层的速度约束有什么关系？

## 实验 4：硬阶跃和光滑轨迹

把 `smoothStep` 目标改成硬阶跃：

```text
targetPosition = time > 0.1 ? 1.2 : 0.0
```

回答：

1. 电流和电压峰值有什么变化？
2. 位置是否更容易超调？
3. 为什么 jerk 连续性会影响底层伺服压力？
