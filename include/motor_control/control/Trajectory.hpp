#pragma once

#include "motor_control/control/FocTransforms.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motor_control::control {

struct TrajectoryPoint {
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

[[nodiscard]] inline double clampValue(double value, double lower, double upper) {
    return std::clamp(value, lower, upper);
}

[[nodiscard]] inline TrajectoryPoint stepTrajectory(double time, double stepTime, double before, double after) {
    return TrajectoryPoint{.position = time >= stepTime ? after : before};
}

[[nodiscard]] inline TrajectoryPoint smoothStepTrajectory(double time,
                                                          double startTime,
                                                          double endTime,
                                                          double startPosition,
                                                          double endPosition) {
    if (endTime <= startTime) {
        throw std::invalid_argument("smoothStepTrajectory requires endTime > startTime");
    }
    if (time <= startTime) {
        return TrajectoryPoint{.position = startPosition};
    }
    if (time >= endTime) {
        return TrajectoryPoint{.position = endPosition};
    }

    const double duration = endTime - startTime;
    const double s = (time - startTime) / duration;
    const double distance = endPosition - startPosition;
    const double blend = s * s * s * (10.0 - 15.0 * s + 6.0 * s * s);
    const double blendDerivative = 30.0 * s * s * (1.0 - s) * (1.0 - s);
    const double blendSecondDerivative = 60.0 * s * (1.0 - s) * (1.0 - 2.0 * s);

    return TrajectoryPoint{
        .position = startPosition + distance * blend,
        .velocity = distance * blendDerivative / duration,
        .acceleration = distance * blendSecondDerivative / (duration * duration),
    };
}

[[nodiscard]] inline TrajectoryPoint sinusoidalTrajectory(double time,
                                                          double amplitude,
                                                          double frequencyHz,
                                                          double offset = 0.0,
                                                          double phase = 0.0) {
    const double omega = 2.0 * PI * frequencyHz;
    const double angle = omega * time + phase;
    return TrajectoryPoint{
        .position = offset + amplitude * std::sin(angle),
        .velocity = amplitude * omega * std::cos(angle),
        .acceleration = -amplitude * omega * omega * std::sin(angle),
    };
}

[[nodiscard]] inline double chirpFrequency(double time, double duration, double startHz, double endHz) {
    if (duration <= 0.0 || startHz <= 0.0 || endHz <= 0.0) {
        throw std::invalid_argument("chirpFrequency requires positive duration and frequencies");
    }
    const double ratio = clampValue(time / duration, 0.0, 1.0);
    return startHz * std::pow(endHz / startHz, ratio);
}

[[nodiscard]] inline TrajectoryPoint chirpTrajectory(double time,
                                                     double duration,
                                                     double amplitude,
                                                     double startHz,
                                                     double endHz) {
    const double frequency = chirpFrequency(time, duration, startHz, endHz);
    return sinusoidalTrajectory(time, amplitude, frequency);
}

} // namespace motor_control::control
