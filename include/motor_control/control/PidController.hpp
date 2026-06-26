#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace motor_control::control {

struct PidGains {
    double kp{0.0};
    double ki{0.0};
    double kd{0.0};
};

struct PidLimits {
    double minOutput{-std::numeric_limits<double>::infinity()};
    double maxOutput{std::numeric_limits<double>::infinity()};
    double minIntegrator{-std::numeric_limits<double>::infinity()};
    double maxIntegrator{std::numeric_limits<double>::infinity()};
};

class PidController {
public:
    PidController() = default;

    PidController(PidGains gains, PidLimits limits = {})
        : _gains(gains), _limits(limits) {
        validateLimits();
    }

    void setGains(PidGains gains) {
        _gains = gains;
    }

    void setLimits(PidLimits limits) {
        _limits = limits;
        validateLimits();
        _integrator = clamp(_integrator, _limits.minIntegrator, _limits.maxIntegrator);
    }

    void reset(double integrator = 0.0) {
        _integrator = clamp(integrator, _limits.minIntegrator, _limits.maxIntegrator);
        _previousMeasurement = 0.0;
        _hasPreviousMeasurement = false;
    }

    [[nodiscard]] double update(double setpoint, double measurement, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("PID update requires positive finite dt");
        }

        const double error = setpoint - measurement;
        _integrator = clamp(_integrator + error * dt, _limits.minIntegrator, _limits.maxIntegrator);

        double derivative = 0.0;
        if (_hasPreviousMeasurement) {
            derivative = -(measurement - _previousMeasurement) / dt;
        }
        _previousMeasurement = measurement;
        _hasPreviousMeasurement = true;

        const double rawOutput = _gains.kp * error + _gains.ki * _integrator + _gains.kd * derivative;
        return clamp(rawOutput, _limits.minOutput, _limits.maxOutput);
    }

    [[nodiscard]] double integrator() const {
        return _integrator;
    }

private:
    static double clamp(double value, double lower, double upper) {
        return std::clamp(value, lower, upper);
    }

    void validateLimits() const {
        if (_limits.minOutput > _limits.maxOutput) {
            throw std::invalid_argument("PID output limits are invalid");
        }
        if (_limits.minIntegrator > _limits.maxIntegrator) {
            throw std::invalid_argument("PID integrator limits are invalid");
        }
    }

    PidGains _gains{};
    PidLimits _limits{};
    double _integrator{0.0};
    double _previousMeasurement{0.0};
    bool _hasPreviousMeasurement{false};
};

} // namespace motor_control::control
