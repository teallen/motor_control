#include "motor_control/control/CascadedServoController.hpp"
#include "motor_control/motor/DcMotor.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <cmath>
#include <iostream>

namespace {

double smoothStep(double time, double startTime, double endTime, double startValue, double endValue) {
    if (time <= startTime) {
        return startValue;
    }
    if (time >= endTime) {
        return endValue;
    }

    const double ratio = (time - startTime) / (endTime - startTime);
    const double blend = ratio * ratio * (3.0 - 2.0 * ratio);
    return startValue + (endValue - startValue) * blend;
}

} // namespace

int main() {
    using motor_control::control::CascadedServoCommand;
    using motor_control::control::CascadedServoConfig;
    using motor_control::control::CascadedServoController;
    using motor_control::control::CascadedServoFeedback;
    using motor_control::control::CascadedServoOutput;
    using motor_control::control::PidGains;
    using motor_control::motor::DcMotor;
    using motor_control::motor::DcMotorParameters;
    using motor_control::utils::CsvLogger;

    DcMotor motor(DcMotorParameters{
        .resistance = 0.9,
        .inductance = 0.001,
        .torqueConstant = 0.09,
        .backEmfConstant = 0.09,
        .rotorInertia = 0.0012,
        .viscousFriction = 0.0015,
        .coulombFriction = 0.003,
    });

    CascadedServoController controller(CascadedServoConfig{
        .positionGains = PidGains{.kp = 10.0, .ki = 0.0, .kd = 0.0},
        .velocityGains = PidGains{.kp = 0.08, .ki = 0.8, .kd = 0.0},
        .currentGains = PidGains{.kp = 1.5, .ki = 120.0, .kd = 0.0},
        .maxVelocity = 6.0,
        .maxCurrent = 5.0,
        .maxVoltage = 24.0,
    });

    CsvLogger logger("outputs/cascaded_servo.csv",
                     {"time_s", "target_position_rad", "position_rad", "velocity_rad_s",
                      "current_a", "velocity_setpoint_rad_s", "current_setpoint_a",
                      "voltage_command_v", "load_torque_nm"});

    constexpr double dt = 0.0001;
    constexpr double duration = 2.0;
    constexpr int logDivider = 10;

    const int steps = static_cast<int>(duration / dt);
    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;
        const double targetPosition = smoothStep(time, 0.1, 0.8, 0.0, 1.2);
        const double loadTorque = time > 1.2 ? 0.025 : 0.0;
        const auto& state = motor.state();

        const CascadedServoOutput output = controller.update(
            CascadedServoCommand{.targetPosition = targetPosition},
            CascadedServoFeedback{.position = state.angularPosition,
                                  .velocity = state.angularVelocity,
                                  .current = state.current},
            dt);

        motor.update(output.voltageCommand, loadTorque, dt);

        if (step % logDivider == 0) {
            const auto& updatedState = motor.state();
            logger.writeRow({time,
                             targetPosition,
                             updatedState.angularPosition,
                             updatedState.angularVelocity,
                             updatedState.current,
                             output.velocitySetpoint,
                             output.currentSetpoint,
                             output.voltageCommand,
                             loadTorque});
        }
    }

    const auto& finalState = motor.state();
    std::cout << "Cascaded servo simulation finished.\n";
    std::cout << "CSV: outputs/cascaded_servo.csv\n";
    std::cout << "Final position(rad): " << finalState.angularPosition << '\n';
    std::cout << "Final current(A): " << finalState.current << '\n';
    return 0;
}
