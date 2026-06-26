# 电机控制学习包

这个仓库面向机械臂规控算法工程师，用 C++ 仿真把“轨迹、关节力矩、电流、驱动器响应”这条链路打通。课程覆盖电机模型、PID、级联伺服、FOC、驱动传感、控制工具、机器人关节、动力学接口、实时安全和调参测试。

## 学习顺序

1. 阅读 [course/00_learning_map.md](course/00_learning_map.md)，明确电机控制和机械臂规划的连接关系。
2. 阅读 [course/01_motor_model_pid.md](course/01_motor_model_pid.md)，运行 DC 电机 PID 位置控制仿真。
3. 阅读 [course/02_cascaded_servo.md](course/02_cascaded_servo.md)，运行位置、速度、电流级联控制仿真。
4. 阅读 [course/03_pmsm_foc.md](course/03_pmsm_foc.md)，运行 PMSM dq 模型和 FOC 电流控制仿真。
5. 阅读 [course/04_drive_sensing.md](course/04_drive_sensing.md)，理解编码器、速度估计、采样和滤波。
6. 阅读 [course/05_control_tools.md](course/05_control_tools.md)，学习阶跃、频响、辨识、前馈和 DOB。
7. 阅读 [course/06_robot_joint_control.md](course/06_robot_joint_control.md)，学习单关节摩擦、重力补偿和阻抗控制。
8. 阅读 [course/07_robot_dynamics_interface.md](course/07_robot_dynamics_interface.md)，把规划轨迹接到 2-DOF computed torque。
9. 阅读 [course/08_realtime_safety.md](course/08_realtime_safety.md)，理解控制周期、延迟、抖动和安全状态机。
10. 阅读 [course/09_tuning_testing.md](course/09_tuning_testing.md)，用测试指标系统化调参。
11. 完成 [exercises/](exercises/) 中的实验任务，重点观察限幅、带宽、负载扰动和采样周期对轨迹跟踪的影响。

## 构建与运行

需要 CMake 3.16+ 和支持 C++20 的编译器。

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build
./build/examples/sim_dc_position_pid
./build/examples/sim_cascaded_servo
./build/examples/sim_pmsm_foc
./build/examples/sim_encoder_velocity_filter
./build/examples/sim_frequency_response
./build/examples/sim_disturbance_observer
./build/examples/sim_single_joint_gravity_friction
./build/examples/sim_joint_impedance
./build/examples/sim_two_link_computed_torque
./build/examples/sim_realtime_jitter_safety
./build/examples/sim_tuning_suite
```

示例程序会在 `outputs/` 下生成 CSV 文件。你可以用 Python、MATLAB、Excel 或其他工具画出位置、速度、电流、电压等曲线。

## 代码结构

- `include/motor_control/control/`：PID、级联控制器、FOC 坐标变换、滤波、轨迹、DOB、阻抗控制。
- `include/motor_control/motor/`：DC 电机模型和 PMSM dq 模型。
- `include/motor_control/robot/`：单关节和 2-DOF 平面臂模型。
- `include/motor_control/system/`：安全状态机。
- `include/motor_control/utils/`：CSV 记录工具。
- `examples/`：可直接运行的仿真实验。
- `tests/`：核心模块的轻量级回归测试。
- `course/`：配套教材。
- `exercises/`：实验任务和思考题。

## 你需要重点建立的直觉

机械臂规划输出的是 `q(t), dq(t), ddq(t)`，驱动器最终执行的是电流控制。中间经过逆动力学、力矩常数、级联伺服环、PWM 和电机本体动态。规划轨迹越接近驱动器的带宽、力矩、电压、电流限制，跟踪误差和发热风险越容易变大。

本学习包的目的不是让你先成为功率硬件专家，而是让你能判断：

- 为什么同样的轨迹在仿真动力学里没问题，上真实关节后可能抖动或跟不上。
- 为什么电流环带宽、速度环带宽、位置环带宽不能随意设置。
- 为什么力矩限幅、电压限幅、anti-windup 会直接影响轨迹跟踪。
- 为什么 FOC 中控制 `Iq` 可以近似理解为控制电机力矩。
