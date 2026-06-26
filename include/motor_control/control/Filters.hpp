#pragma once

#include <cmath>
#include <stdexcept>

namespace motor_control::control {

class LowPassFilter {
public:
    LowPassFilter(double cutoffHz, double initialValue = 0.0)
        : _cutoffHz(cutoffHz), _value(initialValue) {
        if (_cutoffHz <= 0.0 || !std::isfinite(_cutoffHz)) {
            throw std::invalid_argument("LowPassFilter cutoff must be positive");
        }
    }

    void reset(double value = 0.0) {
        _value = value;
        _isInitialized = true;
    }

    [[nodiscard]] double update(double input, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("LowPassFilter update requires positive finite dt");
        }
        if (!_isInitialized) {
            reset(input);
            return _value;
        }

        const double rc = 1.0 / (2.0 * 3.14159265358979323846 * _cutoffHz);
        const double alpha = dt / (rc + dt);
        _value += alpha * (input - _value);
        return _value;
    }

    [[nodiscard]] double value() const {
        return _value;
    }

private:
    double _cutoffHz{1.0};
    double _value{0.0};
    bool _isInitialized{false};
};

class VelocityEstimator {
public:
    explicit VelocityEstimator(double filterCutoffHz = 80.0)
        : _filter(filterCutoffHz) {}

    void reset(double position = 0.0) {
        _previousPosition = position;
        _filter.reset(0.0);
        _hasPreviousPosition = true;
    }

    [[nodiscard]] double update(double position, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("VelocityEstimator update requires positive finite dt");
        }
        if (!_hasPreviousPosition) {
            reset(position);
            return 0.0;
        }

        const double rawVelocity = (position - _previousPosition) / dt;
        _previousPosition = position;
        _rawVelocity = rawVelocity;
        _filteredVelocity = _filter.update(rawVelocity, dt);
        return _filteredVelocity;
    }

    [[nodiscard]] double rawVelocity() const {
        return _rawVelocity;
    }

    [[nodiscard]] double filteredVelocity() const {
        return _filteredVelocity;
    }

private:
    LowPassFilter _filter;
    double _previousPosition{0.0};
    double _rawVelocity{0.0};
    double _filteredVelocity{0.0};
    bool _hasPreviousPosition{false};
};

[[nodiscard]] inline double quantizePosition(double position, int countsPerRevolution) {
    if (countsPerRevolution <= 0) {
        throw std::invalid_argument("countsPerRevolution must be positive");
    }

    constexpr double TWO_PI = 6.28318530717958647692;
    const double counts = std::round(position / TWO_PI * static_cast<double>(countsPerRevolution));
    return counts / static_cast<double>(countsPerRevolution) * TWO_PI;
}

} // namespace motor_control::control
