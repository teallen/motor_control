# 01 电机模型与 PID 练习

对应教材：[course/01_motor_model_pid.md](../course/01_motor_model_pid.md)

## 实验 1：位置阶跃响应

运行：

```bash
./build/examples/sim_dc_position_pid
```

画出：

- `target_position_rad`
- `position_rad`
- `velocity_rad_s`
- `current_a`
- `voltage_v`

回答：

1. 超调大约是多少？
2. 稳定到目标附近大约需要多久？
3. 电压是否长时间贴近 ±24 V？

## 实验 2：比例增益

修改 [examples/sim_dc_position_pid.cpp](../examples/sim_dc_position_pid.cpp) 中的 `kp`：

```text
28 -> 10 -> 60
```

回答：

1. 响应速度如何变化？
2. 超调和振荡如何变化？
3. 电流峰值是否明显变化？

## 实验 3：积分项和负载扰动

把 `ki` 改成 0，再运行一次。重点观察 `time > 1.0` 后负载扰动进入时的位置误差。

回答：

1. 没有积分项时，负载扰动后是否存在稳态误差？
2. 恢复 `ki = 10` 后，稳态误差如何变化？
3. 如果积分限幅太大，可能出现什么问题？

## 实验 4：类比机械臂负载变重

把 `rotorInertia` 放大 5 倍。

回答：

1. 同样 PID 参数下，位置跟踪是否变慢？
2. 电流和电压峰值是否变大？
3. 如果这是机械臂末端拿起重物，你会在规划层做什么限制？
