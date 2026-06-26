# 09 系统调参与测试练习

对应教材：[course/09_tuning_testing.md](../course/09_tuning_testing.md)

## 实验 1：统一测试套件

运行：

```bash
./build/examples/sim_tuning_suite
```

画出：

- `target_position_rad`
- `position_rad`
- `tracking_error_rad`
- `command_torque_nm`
- `external_torque_nm`
- `thermal_index`

回答：

1. 最大误差出现在轨迹加速阶段还是扰动阶段？
2. `thermal_index` 为什么持续上升？
3. 如果只看最终位置，会漏掉哪些问题？

## 实验 2：刚度与热负载

把 [examples/sim_tuning_suite.cpp](../examples/sim_tuning_suite.cpp) 中阻抗刚度从 50 改成 25 和 90。

回答：

1. 刚度提高后最大误差是否降低？
2. 力矩峰值和 `thermal_index` 是否增大？
3. 为什么不能只追求误差最小？

## 实验 3：扰动测试

把外部扰动力矩从 0.7 Nm 改成 1.5 Nm。

回答：

1. 控制器是否触及力矩限幅？
2. 扰动结束后恢复是否有过冲？
3. 如果真实关节频繁遇到这类扰动，应该改控制器、轨迹还是机械设计？

## 实验 4：形成你的调参记录

用以下模板记录一次参数修改：

```text
参数修改：
预期现象：
最大误差：
最大力矩：
热负载指标：
是否触发限幅：
是否值得保留：
```
