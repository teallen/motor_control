#include "motor_control/control/ImpedanceController.hpp"
#include "motor_control/control/Trajectory.hpp"
#include "motor_control/robot/SingleJoint.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <iostream>

int main() {
    using motor_control::control::ImpedanceConfig;
    using motor_control::control::ImpedanceController;
    using motor_control::control::smoothStepTrajectory;
    using motor_control::robot::SingleJoint;
    using motor_control::utils::CsvLogger;

    SingleJoint feedbackOnly;
    SingleJoint withCompensation;
    ImpedanceController controller(ImpedanceConfig{.stiffness = 45.0, .damping = 7.0, .maxTorque = 8.0});

    CsvLogger logger("outputs/single_joint_gravity_friction.csv",
                     {"time_s", "target_position_rad", "target_velocity_rad_s",
                      "position_feedback_only_rad", "position_compensated_rad",
                      "error_feedback_only_rad", "error_compensated_rad",
                      "torque_feedback_only_nm", "torque_compensated_nm",
                      "gravity_feedforward_nm", "friction_feedforward_nm"});

    constexpr double dt = 0.001;
    constexpr double duration = 3.0;

    for (double time = 0.0; time <= duration; time += dt) {
        const auto target = smoothStepTrajectory(time, 0.2, 1.8, 0.0, 0.9);
        const auto& stateFeedbackOnly = feedbackOnly.state();
        const auto& stateCompensated = withCompensation.state();

        const double torqueFeedbackOnly =
            controller.update(target.position, target.velocity, stateFeedbackOnly.position,
                              stateFeedbackOnly.velocity);
        const double gravityFeedforward = withCompensation.gravityTorque(target.position);
        const double frictionFeedforward = withCompensation.frictionTorque(target.velocity);
        const double torqueCompensated =
            controller.update(target.position, target.velocity, stateCompensated.position,
                              stateCompensated.velocity,
                              gravityFeedforward + frictionFeedforward);

        feedbackOnly.update(torqueFeedbackOnly, 0.0, dt);
        withCompensation.update(torqueCompensated, 0.0, dt);

        logger.writeRow({time,
                         target.position,
                         target.velocity,
                         feedbackOnly.state().position,
                         withCompensation.state().position,
                         target.position - feedbackOnly.state().position,
                         target.position - withCompensation.state().position,
                         torqueFeedbackOnly,
                         torqueCompensated,
                         gravityFeedforward,
                         frictionFeedforward});
    }

    std::cout << "Single joint gravity/friction simulation finished.\n";
    std::cout << "CSV: outputs/single_joint_gravity_friction.csv\n";
    return 0;
}
