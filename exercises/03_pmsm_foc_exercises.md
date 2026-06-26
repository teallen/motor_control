# 03 PMSM FOC 练习

对应教材：[course/03_pmsm_foc.md](../course/03_pmsm_foc.md)

## 实验 1：Iq 与力矩

运行：

```bash
./build/examples/sim_pmsm_foc
```

画出：

- `iq_ref_a` 和 `iq_a`
- `torque_nm`
- `mechanical_velocity_rad_s`
- `vd_v` 和 `vq_v`

回答：

1. `Iq` 阶跃后，电磁力矩是否按比例变化？
2. 速度上升后，`Vq` 是否增大？
3. 负载扰动进入后，`Iq` 是否回到目标值？

## 实验 2：电压裕度

把 `maxVoltage` 从 36 V 改成 18 V。

回答：

1. 高速后 `Iq` 跟踪是否变差？
2. `Vq` 是否更容易贴近限幅？
3. 这和机械臂高速运动时力矩输出下降有什么关系？

## 实验 3：改变力矩常数

修改 `fluxLinkage` 或 `polePairs`，比较同样 `Iq` 下的 `torque_nm`。

回答：

1. `fluxLinkage` 增大时，同样 `Iq` 的力矩如何变化？
2. `polePairs` 增大时，同样机械速度下电角速度如何变化？
3. 电角速度变化会如何影响电压需求？

## 实验 4：去掉 dq 解耦

在 [examples/sim_pmsm_foc.cpp](../examples/sim_pmsm_foc.cpp) 中，把电压命令改成只使用 PID 输出，不加解耦项。

回答：

1. `Id` 是否更容易偏离 0？
2. `Iq` 跟踪是否变差？
3. 为什么实际驱动器里会做交叉耦合补偿？
