# 03 PMSM 与 FOC 基础

现代机器人关节常用 PMSM 或 BLDC。对规控算法工程师来说，FOC 最重要的直觉是：通过坐标变换把三相电流变成旋转坐标系下的 `Id` 和 `Iq`，再把 `Iq` 当成主要力矩通道来控制。

## 学习目标

学完本章后，你应该能判断：

- Clarke 和 Park 变换大致在做什么。
- 为什么 `Id=0` 是表贴式 PMSM 常见的基础控制策略。
- 为什么 `Iq` 可以近似理解为力矩指令。
- 为什么高速下需要更多 `Vq` 抵消反电动势，电压裕度会变小。
- 为什么驱动器电流环的性能会影响机械臂力矩控制质量。

## 从 abc 到 dq

三相量 `abc` 是定子三相坐标，随时间正弦变化。FOC 通常做两步变换：

```text
abc -> alpha beta -> dq
```

- Clarke 变换：把三相静止坐标变成二维静止坐标 `alpha beta`。
- Park 变换：用转子电角度把 `alpha beta` 旋转到 `dq` 坐标。

在 `dq` 坐标中，如果角度估计准确，稳态正弦交流量会变成近似直流量，因此可以用 PI/PID 控制器控制。

代码对应 [include/motor_control/control/FocTransforms.hpp](../include/motor_control/control/FocTransforms.hpp)。

## PMSM dq 模型

简化 PMSM dq 电气模型：

```text
Vd = R Id + Ld dId/dt - we Lq Iq
Vq = R Iq + Lq dIq/dt + we (Ld Id + flux)
```

电磁力矩：

```text
Te = 1.5 * pole_pairs * (flux * Iq + (Ld - Lq) * Id * Iq)
```

对于表贴式 PMSM，通常 `Ld` 接近 `Lq`，磁阻转矩项较小，所以可以近似看成：

```text
Te ≈ 1.5 * pole_pairs * flux * Iq
```

这就是“控制 `Iq` 近似等于控制力矩”的来源。

代码对应 [include/motor_control/motor/PmsmDqModel.hpp](../include/motor_control/motor/PmsmDqModel.hpp)。

## 本章仿真做了什么

示例程序 [examples/sim_pmsm_foc.cpp](../examples/sim_pmsm_foc.cpp) 做了一个简化 FOC 电流环：

```text
Id_ref = 0
Iq_ref = 2.5 A
Id/Iq PID -> Vd/Vq
加入 dq 解耦项
限制电压矢量幅值
PMSM dq 模型更新电流、速度、位置
```

它不是完整驱动器固件：没有 SVPWM、死区补偿、采样同步、ADC 噪声或编码器误差。但它足够展示 FOC 中最关键的控制结构。

## 运行实验

```bash
cmake --build build
./build/examples/sim_pmsm_foc
```

输出文件：

```text
outputs/pmsm_foc.csv
```

建议画这些列：

- `id_ref_a` 和 `id_a`：看 d 轴是否被压在 0 附近。
- `iq_ref_a` 和 `iq_a`：看力矩电流跟踪。
- `torque_nm`：看电流到力矩的比例关系。
- `mechanical_velocity_rad_s`：看电机加速过程。
- `vd_v` 和 `vq_v`：看 dq 电压需求。
- `va_v, vb_v, vc_v`：看逆变换后的三相电压指令。

## 和机械臂力矩控制的关系

机械臂动力学控制常会算出期望关节力矩 `tau_joint`。经过减速比 `N` 和效率近似后，可得到电机侧力矩，再换算为 `Iq_ref`：

```text
tau_motor ≈ tau_joint / N
Iq_ref ≈ tau_motor / Kt
```

如果驱动器内部 FOC 电流环跟不上，外部算法给再好的力矩指令也无法准确落地。表现可能是：

- 阻抗控制手感变差。
- 接触力控制有延迟或振荡。
- 重力补偿低速还可以，高速或大负载下误差变大。
- 轨迹拐点附近力矩需求尖峰触发限流。

## 实验建议

1. 把 `iqReference` 从 2.5 A 改到 5 A，观察力矩和加速度变化。
2. 把 `maxVoltage` 从 36 V 改到 18 V，观察高速后 `Iq` 是否更难跟踪。
3. 去掉示例里的 dq 解耦项，比较 `Id` 和 `Iq` 的耦合程度。
4. 把 `loadTorque` 加大，观察负载进入后 `Iq` 是否回到目标值。
5. 修改 `polePairs` 或 `fluxLinkage`，观察同样 `Iq` 下力矩常数如何变化。

## 常见坑

- Park 变换角度错了，`Id/Iq` 会互相串扰，力矩控制会明显变差。
- 只看电流环误差，不看电压限幅，容易忽略高速弱磁或母线电压不足问题。
- `Iq` 到关节力矩还要经过减速器效率、摩擦、间隙和柔性，不能在负载侧完全等同。
- 对机器人关节来说，电流环稳定只是底线，机械结构和外环参数仍然可能引入振荡。
