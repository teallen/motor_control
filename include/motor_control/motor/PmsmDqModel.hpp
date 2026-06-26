#pragma once

#include "motor_control/control/FocTransforms.hpp"

#include <cmath>
#include <stdexcept>

namespace motor_control::motor {

struct PmsmDqParameters {
    double statorResistance{0.35}; // Ohm
    double dAxisInductance{0.00025};
    double qAxisInductance{0.00025};
    double fluxLinkage{0.018};     // Wb
    double polePairs{7.0};
    double rotorInertia{0.00015};  // kg*m^2
    double viscousFriction{0.0002};
};

struct PmsmDqState {
    double dAxisCurrent{0.0};          // A
    double qAxisCurrent{0.0};          // A
    double mechanicalVelocity{0.0};    // rad/s
    double mechanicalPosition{0.0};    // rad
    double electricalAngle{0.0};       // rad
};

class PmsmDqModel {
public:
    explicit PmsmDqModel(PmsmDqParameters parameters = {})
        : _parameters(parameters) {
        validateParameters();
    }

    void reset(PmsmDqState state = {}) {
        _state = state;
        _state.electricalAngle =
            control::normalizeAngle(_parameters.polePairs * _state.mechanicalPosition);
    }

    void update(double dAxisVoltage, double qAxisVoltage, double loadTorque, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("PMSM dq update requires positive finite dt");
        }

        const double electricalVelocity = _parameters.polePairs * _state.mechanicalVelocity;
        const double idDerivative =
            (dAxisVoltage - _parameters.statorResistance * _state.dAxisCurrent +
             electricalVelocity * _parameters.qAxisInductance * _state.qAxisCurrent) /
            _parameters.dAxisInductance;
        const double iqDerivative =
            (qAxisVoltage - _parameters.statorResistance * _state.qAxisCurrent -
             electricalVelocity *
                 (_parameters.dAxisInductance * _state.dAxisCurrent + _parameters.fluxLinkage)) /
            _parameters.qAxisInductance;

        const double acceleration =
            (electromagneticTorque() - loadTorque -
             _parameters.viscousFriction * _state.mechanicalVelocity) /
            _parameters.rotorInertia;

        _state.dAxisCurrent += idDerivative * dt;
        _state.qAxisCurrent += iqDerivative * dt;
        _state.mechanicalVelocity += acceleration * dt;
        _state.mechanicalPosition += _state.mechanicalVelocity * dt;
        _state.electricalAngle =
            control::normalizeAngle(_parameters.polePairs * _state.mechanicalPosition);
    }

    [[nodiscard]] const PmsmDqState& state() const {
        return _state;
    }

    [[nodiscard]] const PmsmDqParameters& parameters() const {
        return _parameters;
    }

    [[nodiscard]] double electromagneticTorque() const {
        return 1.5 * _parameters.polePairs *
               (_parameters.fluxLinkage * _state.qAxisCurrent +
                (_parameters.dAxisInductance - _parameters.qAxisInductance) *
                    _state.dAxisCurrent * _state.qAxisCurrent);
    }

private:
    void validateParameters() const {
        if (_parameters.statorResistance <= 0.0 || _parameters.dAxisInductance <= 0.0 ||
            _parameters.qAxisInductance <= 0.0 || _parameters.fluxLinkage <= 0.0 ||
            _parameters.polePairs <= 0.0 || _parameters.rotorInertia <= 0.0 ||
            _parameters.viscousFriction < 0.0) {
            throw std::invalid_argument("PMSM dq parameters must be physically valid");
        }
    }

    PmsmDqParameters _parameters{};
    PmsmDqState _state{};
};

} // namespace motor_control::motor
