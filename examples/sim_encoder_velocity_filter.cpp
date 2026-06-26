#include "motor_control/control/Filters.hpp"
#include "motor_control/control/Trajectory.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <cmath>
#include <iostream>

int main() {
    using motor_control::control::VelocityEstimator;
    using motor_control::control::quantizePosition;
    using motor_control::control::sinusoidalTrajectory;
    using motor_control::utils::CsvLogger;

    CsvLogger logger("outputs/encoder_velocity_filter.csv",
                     {"time_s", "true_position_rad", "measured_position_rad", "true_velocity_rad_s",
                      "raw_velocity_rad_s", "filtered_velocity_rad_s", "position_error_rad"});

    VelocityEstimator estimator(35.0);
    constexpr double dt = 0.001;
    constexpr double duration = 3.0;
    constexpr int countsPerRevolution = 4096;

    for (double time = 0.0; time <= duration; time += dt) {
        const auto truth = sinusoidalTrajectory(time, 0.8, 0.8, 0.2);
        const double deterministicNoise = 0.0008 * std::sin(2.0 * motor_control::control::PI * 37.0 * time);
        const double measuredPosition =
            quantizePosition(truth.position + deterministicNoise, countsPerRevolution);
        const double filteredVelocity = estimator.update(measuredPosition, dt);

        logger.writeRow({time,
                         truth.position,
                         measuredPosition,
                         truth.velocity,
                         estimator.rawVelocity(),
                         filteredVelocity,
                         measuredPosition - truth.position});
    }

    std::cout << "Encoder velocity filter simulation finished.\n";
    std::cout << "CSV: outputs/encoder_velocity_filter.csv\n";
    return 0;
}
