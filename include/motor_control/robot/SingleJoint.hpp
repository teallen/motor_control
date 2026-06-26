#pragma once

#include "motor_control/control/Trajectory.hpp"

#include <cmath>
#include <stdexcept>

namespace motor_control::robot {

struct SingleJointParameters {
    double inertia{0.08};
    double damping{0.04};
    double coulombFriction{0.12};
    double gravityTorqueAmplitude{1.6};
    double gearRatio{80.0};
    double torqueLimit{8.0};
};

struct SingleJointState {
    double position{0.0};
    double velocity{0.0};
    double acceleration{0.0};
};

class SingleJoint {
public:
    explicit SingleJoint(SingleJointParameters parameters = {})
        : _parameters(parameters) {
        validateParameters();
    }

    void reset(SingleJointState state = {}) {
        _state = state;
    }

    void update(double commandedJointTorque, double externalTorque, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("SingleJoint update requires positive finite dt");
        }

        const double limitedTorque =
            control::clampValue(commandedJointTorque, -_parameters.torqueLimit, _parameters.torqueLimit);
        const double netTorque = limitedTorque - gravityTorque(_state.position) -
                                 frictionTorque(_state.velocity) - externalTorque;
        _state.acceleration = netTorque / _parameters.inertia;
        _state.velocity += _state.acceleration * dt;
        _state.position += _state.velocity * dt;
    }

    [[nodiscard]] const SingleJointState& state() const {
        return _state;
    }

    [[nodiscard]] const SingleJointParameters& parameters() const {
        return _parameters;
    }

    [[nodiscard]] double gravityTorque(double position) const {
        return _parameters.gravityTorqueAmplitude * std::sin(position);
    }

    [[nodiscard]] double frictionTorque(double velocity) const {
        double friction = _parameters.damping * velocity;
        if (std::abs(velocity) > 1e-5) {
            friction += _parameters.coulombFriction * (velocity > 0.0 ? 1.0 : -1.0);
        }
        return friction;
    }

    [[nodiscard]] double motorTorqueToJointTorque(double motorTorque) const {
        return motorTorque * _parameters.gearRatio;
    }

    [[nodiscard]] double jointTorqueToMotorTorque(double jointTorque) const {
        return jointTorque / _parameters.gearRatio;
    }

private:
    void validateParameters() const {
        if (_parameters.inertia <= 0.0 || _parameters.damping < 0.0 ||
            _parameters.coulombFriction < 0.0 || _parameters.gearRatio <= 0.0 ||
            _parameters.torqueLimit <= 0.0) {
            throw std::invalid_argument("SingleJoint parameters must be valid");
        }
    }

    SingleJointParameters _parameters{};
    SingleJointState _state{};
};

} // namespace motor_control::robot
