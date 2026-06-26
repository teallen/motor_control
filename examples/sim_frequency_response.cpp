#include "motor_control/control/Trajectory.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

struct FrequencyResult {
    double frequencyHz{0.0};
    double gain{0.0};
    double phaseDeg{0.0};
};

FrequencyResult estimateFirstOrderClosedLoop(double frequencyHz, double bandwidthHz) {
    constexpr double dt = 0.0005;
    constexpr double settleCycles = 3.0;
    constexpr double measureCycles = 5.0;
    const double omega = 2.0 * motor_control::control::PI * frequencyHz;
    const double tau = 1.0 / (2.0 * motor_control::control::PI * bandwidthHz);
    const double duration = (settleCycles + measureCycles) / frequencyHz;
    const int steps = static_cast<int>(duration / dt);

    double output = 0.0;
    double sinProjection = 0.0;
    double cosProjection = 0.0;
    int measuredSamples = 0;

    for (int step = 0; step <= steps; ++step) {
        const double time = static_cast<double>(step) * dt;
        const double input = std::sin(omega * time);
        output += ((input - output) / tau) * dt;

        if (time > settleCycles / frequencyHz) {
            sinProjection += output * std::sin(omega * time);
            cosProjection += output * std::cos(omega * time);
            ++measuredSamples;
        }
    }

    const double sineCoefficient = 2.0 * sinProjection / static_cast<double>(measuredSamples);
    const double cosineCoefficient = 2.0 * cosProjection / static_cast<double>(measuredSamples);
    const double gain = std::hypot(sineCoefficient, cosineCoefficient);
    const double phase = std::atan2(cosineCoefficient, sineCoefficient) * 180.0 / motor_control::control::PI;
    return FrequencyResult{.frequencyHz = frequencyHz, .gain = gain, .phaseDeg = phase};
}

} // namespace

int main() {
    using motor_control::utils::CsvLogger;

    CsvLogger logger("outputs/frequency_response.csv", {"frequency_hz", "gain", "phase_deg"});
    const std::vector<double> frequencies{0.5, 1.0, 2.0, 4.0, 8.0, 12.0, 20.0, 35.0};
    constexpr double bandwidthHz = 8.0;

    for (const double frequency : frequencies) {
        const auto result = estimateFirstOrderClosedLoop(frequency, bandwidthHz);
        logger.writeRow({result.frequencyHz, result.gain, result.phaseDeg});
    }

    std::cout << "Frequency response simulation finished.\n";
    std::cout << "CSV: outputs/frequency_response.csv\n";
    return 0;
}
