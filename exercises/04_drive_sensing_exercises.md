# 04 驱动器与传感器练习

对应教材：[course/04_drive_sensing.md](../course/04_drive_sensing.md)

## 实验 1：编码器量化与速度估计

运行：

```bash
./build/examples/sim_encoder_velocity_filter
```

画出：

- `true_velocity_rad_s`
- `raw_velocity_rad_s`
- `filtered_velocity_rad_s`
- `position_error_rad`

回答：

1. 原始差分速度是否明显抖动？
2. 滤波后的速度相对真值是否有延迟？
3. 速度噪声如果进入速度环，会变成什么控制现象？

## 实验 2：编码器分辨率

把 `countsPerRevolution` 从 4096 改成 1024，再改成 16384。

回答：

1. 低分辨率时速度估计有什么变化？
2. 高分辨率是否完全消除速度噪声？
3. 真实系统还会有哪些非量化误差？

## 实验 3：滤波带宽

把 `VelocityEstimator estimator(35.0)` 改成 10 Hz 和 100 Hz。

回答：

1. 低截止频率下曲线是否更平滑？
2. 低截止频率下速度响应是否更滞后？
3. 速度环带宽和速度滤波截止频率应该如何匹配？

## 实验 4：联系规划轨迹

把正弦频率从 0.8 Hz 改成 2 Hz。

回答：

1. 同样滤波参数下，速度估计误差是否变大？
2. 如果这是机械臂关节快速往复运动，规划层应如何限制速度、加速度或 jerk？
