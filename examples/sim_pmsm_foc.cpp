#include "motor_control/control/FocTransforms.hpp"
#include "motor_control/control/PidController.hpp"
#include "motor_control/motor/PmsmDqModel.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <cmath>
#include <iostream>

namespace {

motor_control::control::Dq limitVoltageVector(motor_control::control::Dq voltage, double maxMagnitude) {
    const double magnitude = std::hypot(voltage.d, voltage.q);
    if (magnitude <= maxMagnitude || magnitude <= 1e-12) {
        return voltage;
    }

    const double scale = maxMagnitude / magnitude;
    voltage.d *= scale;
    voltage.q *= scale;
    return voltage;
}

} // namespace

int main() {
    using motor_control::control::Dq;
    using motor_control::control::inverseClarkeTransform;
    using motor_control::control::inverseParkTransform;
    using motor_control::control::PidController;
    using motor_control::control::PidGains;
    using motor_control::control::PidLimits;
    using motor_control::motor::PmsmDqModel;
    using motor_control::motor::PmsmDqParameters;
    using motor_control::utils::CsvLogger;

    PmsmDqModel motor(PmsmDqParameters{
        .statorResistance = 0.32,
        .dAxisInductance = 0.00028,
        .qAxisInductance = 0.00030,
        .fluxLinkage = 0.016,
        .polePairs = 7.0,
        .rotorInertia = 0.00018,
        .viscousFriction = 0.00025,
    });

    constexpr double maxVoltage = 36.0;
    PidController dAxisCurrentLoop(
        PidGains{.kp = 2.0, .ki = 1800.0, .kd = 0.0},
        PidLimits{.minOutput = -maxVoltage, .maxOutput = maxVoltage, .minIntegrator = -0.05, .maxIntegrator = 0.05});
    PidController qAxisCurrentLoop(
        PidGains{.kp = 2.0, .ki = 1800.0, .kd = 0.0},
        PidLimits{.minOutput = -maxVoltage, .maxOutput = maxVoltage, .minIntegrator = -0.05, .maxIntegrator = 0.05});

    CsvLogger logger("outputs/pmsm_foc.csv",
                     {"time_s", "id_ref_a", "iq_ref_a", "id_a", "iq_a", "torque_nm",
                      "mechanical_velocity_rad_s", "mechanical_position_rad",
                      "vd_v", "vq_v", "va_v", "vb_v", "vc_v", "load_torque_nm"});

    constexpr double dt = 0.00001;
    constexpr double duration = 0.12;
    constexpr int logDivider = 10;
    const auto& parameters = motor.parameters();

    const int steps = static_cast<int>(duration / dt);
    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;
        const double idReference = 0.0;
        const double iqReference = time > 0.02 ? 2.5 : 0.0;
        const double loadTorque = time > 0.08 ? 0.16 : 0.0;
        const auto& state = motor.state();

        const double electricalVelocity = parameters.polePairs * state.mechanicalVelocity;
        const double vdFeedback = dAxisCurrentLoop.update(idReference, state.dAxisCurrent, dt);
        const double vqFeedback = qAxisCurrentLoop.update(iqReference, state.qAxisCurrent, dt);

        // Decoupling terms keep the dq current loops easier to tune at non-zero speed.
        Dq voltageCommand{
            .d = vdFeedback - electricalVelocity * parameters.qAxisInductance * state.qAxisCurrent,
            .q = vqFeedback +
                 electricalVelocity *
                     (parameters.dAxisInductance * state.dAxisCurrent + parameters.fluxLinkage),
        };
        voltageCommand = limitVoltageVector(voltageCommand, maxVoltage);

        motor.update(voltageCommand.d, voltageCommand.q, loadTorque, dt);

        if (step % logDivider == 0) {
            const auto& updatedState = motor.state();
            const auto phaseVoltage =
                inverseClarkeTransform(inverseParkTransform(voltageCommand, updatedState.electricalAngle));

            logger.writeRow({time,
                             idReference,
                             iqReference,
                             updatedState.dAxisCurrent,
                             updatedState.qAxisCurrent,
                             motor.electromagneticTorque(),
                             updatedState.mechanicalVelocity,
                             updatedState.mechanicalPosition,
                             voltageCommand.d,
                             voltageCommand.q,
                             phaseVoltage.a,
                             phaseVoltage.b,
                             phaseVoltage.c,
                             loadTorque});
        }
    }

    const auto& finalState = motor.state();
    std::cout << "PMSM FOC simulation finished.\n";
    std::cout << "CSV: outputs/pmsm_foc.csv\n";
    std::cout << "Final iq(A): " << finalState.qAxisCurrent << '\n';
    std::cout << "Final torque(Nm): " << motor.electromagneticTorque() << '\n';
    return 0;
}
