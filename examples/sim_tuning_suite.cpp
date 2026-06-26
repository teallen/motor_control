#include "motor_control/control/ImpedanceController.hpp"
#include "motor_control/control/Trajectory.hpp"
#include "motor_control/robot/SingleJoint.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

int main() {
    using motor_control::control::ImpedanceConfig;
    using motor_control::control::ImpedanceController;
    using motor_control::control::smoothStepTrajectory;
    using motor_control::robot::SingleJoint;
    using motor_control::utils::CsvLogger;

    SingleJoint joint;
    ImpedanceController controller(ImpedanceConfig{.stiffness = 50.0, .damping = 7.0, .maxTorque = 8.0});

    CsvLogger logger("outputs/tuning_suite.csv",
                     {"time_s", "target_position_rad", "position_rad", "velocity_rad_s",
                      "tracking_error_rad", "command_torque_nm", "external_torque_nm",
                      "absolute_error_rad", "thermal_index"});

    constexpr double dt = 0.001;
    constexpr double duration = 4.0;
    double integralSquaredTorque = 0.0;
    double maxAbsError = 0.0;

    for (double time = 0.0; time <= duration; time += dt) {
        const auto target = time < 2.0 ? smoothStepTrajectory(time, 0.2, 1.4, 0.0, 0.8)
                                       : smoothStepTrajectory(time, 2.0, 3.2, 0.8, -0.3);
        const double externalTorque = (time > 2.5 && time < 3.0) ? 0.7 : 0.0;
        const auto& state = joint.state();
        const double gravityFeedforward = joint.gravityTorque(target.position);
        const double torque = controller.update(target.position, target.velocity, state.position,
                                                state.velocity, gravityFeedforward);

        joint.update(torque, externalTorque, dt);

        const double error = target.position - joint.state().position;
        maxAbsError = std::max(maxAbsError, std::abs(error));
        integralSquaredTorque += torque * torque * dt;
        logger.writeRow({time,
                         target.position,
                         joint.state().position,
                         joint.state().velocity,
                         error,
                         torque,
                         externalTorque,
                         std::abs(error),
                         integralSquaredTorque});
    }

    std::cout << "Tuning suite simulation finished.\n";
    std::cout << "CSV: outputs/tuning_suite.csv\n";
    std::cout << "Max absolute error(rad): " << maxAbsError << '\n';
    std::cout << "Thermal index(integral torque^2): " << integralSquaredTorque << '\n';
    return 0;
}
