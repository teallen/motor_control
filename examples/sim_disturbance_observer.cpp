#include "motor_control/control/DisturbanceObserver.hpp"
#include "motor_control/control/PidController.hpp"
#include "motor_control/robot/SingleJoint.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <iostream>

int main() {
    using motor_control::control::DisturbanceObserver;
    using motor_control::control::PidController;
    using motor_control::control::PidGains;
    using motor_control::control::PidLimits;
    using motor_control::robot::SingleJoint;
    using motor_control::robot::SingleJointParameters;
    using motor_control::utils::CsvLogger;

    SingleJoint jointWithoutDob(SingleJointParameters{.gravityTorqueAmplitude = 0.0, .torqueLimit = 6.0});
    SingleJoint jointWithDob(SingleJointParameters{.gravityTorqueAmplitude = 0.0, .torqueLimit = 6.0});
    PidController pidWithoutDob(PidGains{.kp = 30.0, .ki = 8.0, .kd = 2.5},
                                PidLimits{.minOutput = -6.0,
                                          .maxOutput = 6.0,
                                          .minIntegrator = -0.5,
                                          .maxIntegrator = 0.5});
    PidController pidWithDob(PidGains{.kp = 30.0, .ki = 8.0, .kd = 2.5},
                             PidLimits{.minOutput = -6.0,
                                       .maxOutput = 6.0,
                                       .minIntegrator = -0.5,
                                       .maxIntegrator = 0.5});
    DisturbanceObserver observer(jointWithDob.parameters().inertia, jointWithDob.parameters().damping, 25.0);

    CsvLogger logger("outputs/disturbance_observer.csv",
                     {"time_s", "target_position_rad", "position_no_dob_rad", "position_dob_rad",
                      "velocity_dob_rad_s", "load_torque_nm", "estimated_disturbance_nm",
                      "torque_no_dob_nm", "torque_dob_nm"});

    constexpr double dt = 0.001;
    constexpr double duration = 3.0;

    for (double time = 0.0; time <= duration; time += dt) {
        const double target = 0.6;
        const double loadTorque = time > 1.0 ? 0.8 : 0.0;

        const auto& stateWithoutDob = jointWithoutDob.state();
        const auto& stateWithDob = jointWithDob.state();
        const double baseTorqueWithoutDob = pidWithoutDob.update(target, stateWithoutDob.position, dt);
        const double baseTorqueWithDob = pidWithDob.update(target, stateWithDob.position, dt);
        const double estimatedDisturbance =
            observer.update(baseTorqueWithDob, stateWithDob.velocity, dt);
        const double compensatedTorque = baseTorqueWithDob + estimatedDisturbance;

        jointWithoutDob.update(baseTorqueWithoutDob, loadTorque, dt);
        jointWithDob.update(compensatedTorque, loadTorque, dt);

        logger.writeRow({time,
                         target,
                         jointWithoutDob.state().position,
                         jointWithDob.state().position,
                         jointWithDob.state().velocity,
                         loadTorque,
                         estimatedDisturbance,
                         baseTorqueWithoutDob,
                         compensatedTorque});
    }

    std::cout << "Disturbance observer simulation finished.\n";
    std::cout << "CSV: outputs/disturbance_observer.csv\n";
    return 0;
}
