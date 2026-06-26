#pragma once

#include "motor_control/control/Filters.hpp"

#include <cmath>
#include <stdexcept>

namespace motor_control::control {

class DisturbanceObserver {
public:
    DisturbanceObserver(double inertia, double damping, double cutoffHz)
        : _inertia(inertia), _damping(damping), _filter(cutoffHz) {
        if (_inertia <= 0.0 || _damping < 0.0) {
            throw std::invalid_argument("DisturbanceObserver parameters must be valid");
        }
    }

    void reset(double velocity = 0.0) {
        _previousVelocity = velocity;
        _hasPreviousVelocity = true;
        _filter.reset(0.0);
        _estimatedDisturbance = 0.0;
    }

    [[nodiscard]] double update(double commandedTorque, double measuredVelocity, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("DisturbanceObserver update requires positive finite dt");
        }
        if (!_hasPreviousVelocity) {
            reset(measuredVelocity);
            return 0.0;
        }

        const double acceleration = (measuredVelocity - _previousVelocity) / dt;
        _previousVelocity = measuredVelocity;
        const double rawDisturbance =
            commandedTorque - _inertia * acceleration - _damping * measuredVelocity;
        _estimatedDisturbance = _filter.update(rawDisturbance, dt);
        return _estimatedDisturbance;
    }

    [[nodiscard]] double estimatedDisturbance() const {
        return _estimatedDisturbance;
    }

private:
    double _inertia{1.0};
    double _damping{0.0};
    LowPassFilter _filter;
    double _previousVelocity{0.0};
    double _estimatedDisturbance{0.0};
    bool _hasPreviousVelocity{false};
};

} // namespace motor_control::control
