# 05 控制工具练习

对应教材：[course/05_control_tools.md](../course/05_control_tools.md)

## 实验 1：频响幅值和相位

运行：

```bash
./build/examples/sim_frequency_response
```

画出：

- `frequency_hz` - `gain`
- `frequency_hz` - `phase_deg`

回答：

1. 频率升高后，增益如何变化？
2. 频率升高后，相位滞后如何变化？
3. 为什么位置环带宽不能接近传感器滤波和通信延迟的极限？

## 实验 2：改变闭环带宽

修改 [examples/sim_frequency_response.cpp](../examples/sim_frequency_response.cpp) 中的 `bandwidthHz`，从 8 Hz 改为 4 Hz 和 16 Hz。

回答：

1. 带宽提高后，哪些频率的跟踪增益改善最明显？
2. 带宽提高是否一定安全？
3. 在真实关节上，带宽提高会放大哪些风险？

## 实验 3：扰动观测器

运行：

```bash
./build/examples/sim_disturbance_observer
```

画出：

- `position_no_dob_rad`
- `position_dob_rad`
- `load_torque_nm`
- `estimated_disturbance_nm`

回答：

1. 负载扰动进入后，有 DOB 的位置误差是否更快恢复？
2. 估计扰动是否存在延迟？
3. 如果速度测量噪声很大，DOB 会出现什么问题？

## 实验 4：DOB 带宽

把 `DisturbanceObserver` 的截止频率从 25 Hz 改成 5 Hz 和 80 Hz。

回答：

1. 低带宽时补偿是否滞后？
2. 高带宽时估计是否更容易抖动？
3. 这和关节力矩控制里的“手感发硬/发抖”有什么关系？
