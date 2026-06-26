#include "motor_control/control/Trajectory.hpp"
#include "motor_control/robot/TwoLinkArm.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <iostream>

int main() {
    using motor_control::control::smoothStepTrajectory;
    using motor_control::robot::JointVector2;
    using motor_control::robot::TwoLinkArm;
    using motor_control::utils::CsvLogger;

    TwoLinkArm arm;

    CsvLogger logger("outputs/two_link_computed_torque.csv",
                     {"time_s", "q1_des_rad", "q2_des_rad", "q1_rad", "q2_rad",
                      "dq1_des_rad_s", "dq2_des_rad_s", "dq1_rad_s", "dq2_rad_s",
                      "q1_error_rad", "q2_error_rad", "tau1_nm", "tau2_nm"});

    constexpr double dt = 0.001;
    constexpr double duration = 3.0;
    constexpr double kp = 60.0;
    constexpr double kd = 14.0;

    for (double time = 0.0; time <= duration; time += dt) {
        const auto q1 = smoothStepTrajectory(time, 0.2, 2.2, 0.0, 0.8);
        const auto q2 = smoothStepTrajectory(time, 0.4, 2.4, 0.0, -0.6);

        const JointVector2 desiredPosition{.first = q1.position, .second = q2.position};
        const JointVector2 desiredVelocity{.first = q1.velocity, .second = q2.velocity};
        const JointVector2 desiredAcceleration{.first = q1.acceleration, .second = q2.acceleration};
        const JointVector2 torque =
            arm.computedTorque(desiredPosition, desiredVelocity, desiredAcceleration, kp, kd);

        arm.update(torque, dt);

        const auto& state = arm.state();
        logger.writeRow({time,
                         q1.position,
                         q2.position,
                         state.position.first,
                         state.position.second,
                         q1.velocity,
                         q2.velocity,
                         state.velocity.first,
                         state.velocity.second,
                         q1.position - state.position.first,
                         q2.position - state.position.second,
                         torque.first,
                         torque.second});
    }

    std::cout << "Two-link computed torque simulation finished.\n";
    std::cout << "CSV: outputs/two_link_computed_torque.csv\n";
    return 0;
}
