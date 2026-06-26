#include "motor_control/control/PidController.hpp"
#include "motor_control/motor/DcMotor.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <iostream>

int main() {
    using motor_control::control::PidController;
    using motor_control::control::PidGains;
    using motor_control::control::PidLimits;
    using motor_control::motor::DcMotor;
    using motor_control::motor::DcMotorParameters;
    using motor_control::utils::CsvLogger;

    DcMotor motor(DcMotorParameters{
        .resistance = 1.1,
        .inductance = 0.0015,
        .torqueConstant = 0.075,
        .backEmfConstant = 0.075,
        .rotorInertia = 0.0009,
        .viscousFriction = 0.0012,
        .coulombFriction = 0.002,
    });

    PidController positionPid(
        PidGains{.kp = 28.0, .ki = 10.0, .kd = 0.9},
        PidLimits{.minOutput = -24.0, .maxOutput = 24.0, .minIntegrator = -1.5, .maxIntegrator = 1.5});

    CsvLogger logger("outputs/dc_position_pid.csv",
                     {"time_s", "target_position_rad", "position_rad", "velocity_rad_s",
                      "current_a", "torque_nm", "voltage_v", "load_torque_nm"});

    constexpr double dt = 0.001;
    constexpr double duration = 2.0;

    for (double time = 0.0; time <= duration; time += dt) {
        const double targetPosition = time > 0.05 ? 1.0 : 0.0;
        const double loadTorque = time > 1.0 ? 0.025 : 0.0;
        const auto& state = motor.state();

        const double voltage = positionPid.update(targetPosition, state.angularPosition, dt);
        motor.update(voltage, loadTorque, dt);

        const auto& updatedState = motor.state();
        logger.writeRow({time,
                         targetPosition,
                         updatedState.angularPosition,
                         updatedState.angularVelocity,
                         updatedState.current,
                         motor.torque(),
                         voltage,
                         loadTorque});
    }

    const auto& finalState = motor.state();
    std::cout << "DC position PID simulation finished.\n";
    std::cout << "CSV: outputs/dc_position_pid.csv\n";
    std::cout << "Final position(rad): " << finalState.angularPosition << '\n';
    std::cout << "Final velocity(rad/s): " << finalState.angularVelocity << '\n';
    return 0;
}
