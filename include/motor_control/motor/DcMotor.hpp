#pragma once

#include <cmath>
#include <stdexcept>

namespace motor_control::motor {

struct DcMotorParameters {
    double resistance{1.2};       // Ohm
    double inductance{0.002};     // H
    double torqueConstant{0.08};  // N*m/A
    double backEmfConstant{0.08}; // V/(rad/s)
    double rotorInertia{0.0008};  // kg*m^2
    double viscousFriction{0.001};
    double coulombFriction{0.0};
};

struct DcMotorState {
    double current{0.0};          // A
    double angularVelocity{0.0};  // rad/s
    double angularPosition{0.0};  // rad
};

class DcMotor {
public:
    explicit DcMotor(DcMotorParameters parameters = {})
        : _parameters(parameters) {
        validateParameters();
    }

    void reset(DcMotorState state = {}) {
        _state = state;
    }

    void update(double terminalVoltage, double loadTorque, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("DC motor update requires positive finite dt");
        }

        const double electricalDerivative =
            (terminalVoltage - _parameters.resistance * _state.current -
             _parameters.backEmfConstant * _state.angularVelocity) /
            _parameters.inductance;

        const double motorTorque = torque();
        const double frictionTorque = calculateFrictionTorque(_state.angularVelocity);
        const double mechanicalDerivative =
            (motorTorque - loadTorque - frictionTorque) / _parameters.rotorInertia;

        _state.current += electricalDerivative * dt;
        _state.angularVelocity += mechanicalDerivative * dt;
        _state.angularPosition += _state.angularVelocity * dt;
    }

    [[nodiscard]] const DcMotorState& state() const {
        return _state;
    }

    [[nodiscard]] const DcMotorParameters& parameters() const {
        return _parameters;
    }

    [[nodiscard]] double torque() const {
        return _parameters.torqueConstant * _state.current;
    }

private:
    void validateParameters() const {
        if (_parameters.resistance <= 0.0 || _parameters.inductance <= 0.0 ||
            _parameters.torqueConstant <= 0.0 || _parameters.backEmfConstant <= 0.0 ||
            _parameters.rotorInertia <= 0.0 || _parameters.viscousFriction < 0.0 ||
            _parameters.coulombFriction < 0.0) {
            throw std::invalid_argument("DC motor parameters must be physically valid");
        }
    }

    double calculateFrictionTorque(double velocity) const {
        double friction = _parameters.viscousFriction * velocity;
        if (std::abs(velocity) > 1e-6) {
            friction += _parameters.coulombFriction * (velocity > 0.0 ? 1.0 : -1.0);
        }
        return friction;
    }

    DcMotorParameters _parameters{};
    DcMotorState _state{};
};

} // namespace motor_control::motor
