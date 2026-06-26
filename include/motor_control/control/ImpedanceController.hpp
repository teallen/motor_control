#pragma once

#include "motor_control/control/Trajectory.hpp"

#include <algorithm>
#include <stdexcept>

namespace motor_control::control {

struct ImpedanceConfig {
    double stiffness{40.0};
    double damping{4.0};
    double maxTorque{20.0};
};

class ImpedanceController {
public:
    explicit ImpedanceController(ImpedanceConfig config = {})
        : _config(config) {
        if (_config.stiffness < 0.0 || _config.damping < 0.0 || _config.maxTorque <= 0.0) {
            throw std::invalid_argument("ImpedanceController parameters must be valid");
        }
    }

    [[nodiscard]] double update(double desiredPosition,
                                double desiredVelocity,
                                double measuredPosition,
                                double measuredVelocity,
                                double torqueFeedforward = 0.0) const {
        const double positionError = desiredPosition - measuredPosition;
        const double velocityError = desiredVelocity - measuredVelocity;
        const double torque = torqueFeedforward + _config.stiffness * positionError +
                              _config.damping * velocityError;
        return clampValue(torque, -_config.maxTorque, _config.maxTorque);
    }

private:
    ImpedanceConfig _config{};
};

struct AdmittanceConfig {
    double virtualMass{1.0};
    double virtualDamping{8.0};
    double virtualStiffness{30.0};
};

class AdmittanceController {
public:
    explicit AdmittanceController(AdmittanceConfig config = {})
        : _config(config) {
        if (_config.virtualMass <= 0.0 || _config.virtualDamping < 0.0 ||
            _config.virtualStiffness < 0.0) {
            throw std::invalid_argument("AdmittanceController parameters must be valid");
        }
    }

    [[nodiscard]] TrajectoryPoint update(double referencePosition, double externalTorque, double dt) {
        if (dt <= 0.0) {
            throw std::invalid_argument("AdmittanceController update requires positive dt");
        }

        const double acceleration =
            (externalTorque - _config.virtualDamping * _state.velocity -
             _config.virtualStiffness * (_state.position - referencePosition)) /
            _config.virtualMass;
        _state.velocity += acceleration * dt;
        _state.position += _state.velocity * dt;
        _state.acceleration = acceleration;
        return _state;
    }

private:
    AdmittanceConfig _config{};
    TrajectoryPoint _state{};
};

} // namespace motor_control::control
