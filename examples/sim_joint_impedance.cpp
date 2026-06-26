#include "motor_control/control/ImpedanceController.hpp"
#include "motor_control/robot/SingleJoint.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <iostream>

int main() {
    using motor_control::control::ImpedanceConfig;
    using motor_control::control::ImpedanceController;
    using motor_control::robot::SingleJoint;
    using motor_control::utils::CsvLogger;

    SingleJoint softJoint;
    SingleJoint stiffJoint;
    ImpedanceController softController(ImpedanceConfig{.stiffness = 18.0, .damping = 3.0, .maxTorque = 8.0});
    ImpedanceController stiffController(ImpedanceConfig{.stiffness = 75.0, .damping = 8.0, .maxTorque = 8.0});

    CsvLogger logger("outputs/joint_impedance.csv",
                     {"time_s", "external_torque_nm", "soft_position_rad", "stiff_position_rad",
                      "soft_velocity_rad_s", "stiff_velocity_rad_s", "soft_torque_nm",
                      "stiff_torque_nm"});

    constexpr double dt = 0.001;
    constexpr double duration = 2.5;
    constexpr double holdPosition = 0.4;

    for (double time = 0.0; time <= duration; time += dt) {
        const double externalTorque = (time > 0.8 && time < 1.6) ? 1.2 : 0.0;
        const auto& softState = softJoint.state();
        const auto& stiffState = stiffJoint.state();

        const double softGravity = softJoint.gravityTorque(holdPosition);
        const double stiffGravity = stiffJoint.gravityTorque(holdPosition);
        const double softTorque =
            softController.update(holdPosition, 0.0, softState.position, softState.velocity, softGravity);
        const double stiffTorque =
            stiffController.update(holdPosition, 0.0, stiffState.position, stiffState.velocity, stiffGravity);

        softJoint.update(softTorque, externalTorque, dt);
        stiffJoint.update(stiffTorque, externalTorque, dt);

        logger.writeRow({time,
                         externalTorque,
                         softJoint.state().position,
                         stiffJoint.state().position,
                         softJoint.state().velocity,
                         stiffJoint.state().velocity,
                         softTorque,
                         stiffTorque});
    }

    std::cout << "Joint impedance simulation finished.\n";
    std::cout << "CSV: outputs/joint_impedance.csv\n";
    return 0;
}
