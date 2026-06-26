#pragma once

#include "motor_control/control/Trajectory.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace motor_control::robot {

struct JointVector2 {
    double first{0.0};
    double second{0.0};
};

struct Matrix2 {
    double m11{0.0};
    double m12{0.0};
    double m21{0.0};
    double m22{0.0};
};

struct TwoLinkParameters {
    double link1Length{0.5};
    double link2Length{0.4};
    double link1Mass{3.0};
    double link2Mass{2.0};
    double link1Inertia{0.06};
    double link2Inertia{0.03};
    double gravity{9.81};
    double jointDamping{0.08};
    double torqueLimit{25.0};
};

struct TwoLinkState {
    JointVector2 position{};
    JointVector2 velocity{};
    JointVector2 acceleration{};
};

class TwoLinkArm {
public:
    explicit TwoLinkArm(TwoLinkParameters parameters = {})
        : _parameters(parameters) {
        validateParameters();
    }

    void reset(TwoLinkState state = {}) {
        _state = state;
    }

    void update(JointVector2 commandedTorque, double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("TwoLinkArm update requires positive finite dt");
        }

        commandedTorque.first =
            control::clampValue(commandedTorque.first, -_parameters.torqueLimit, _parameters.torqueLimit);
        commandedTorque.second =
            control::clampValue(commandedTorque.second, -_parameters.torqueLimit, _parameters.torqueLimit);

        const Matrix2 mass = massMatrix(_state.position);
        const JointVector2 coriolis = coriolisVector(_state.position, _state.velocity);
        const JointVector2 gravityVector = gravityTorque(_state.position);
        const JointVector2 damping{.first = _parameters.jointDamping * _state.velocity.first,
                                   .second = _parameters.jointDamping * _state.velocity.second};

        const JointVector2 rhs{.first = commandedTorque.first - coriolis.first - gravityVector.first - damping.first,
                               .second = commandedTorque.second - coriolis.second - gravityVector.second -
                                         damping.second};
        _state.acceleration = solveMassMatrix(mass, rhs);
        _state.velocity.first += _state.acceleration.first * dt;
        _state.velocity.second += _state.acceleration.second * dt;
        _state.position.first += _state.velocity.first * dt;
        _state.position.second += _state.velocity.second * dt;
    }

    [[nodiscard]] const TwoLinkState& state() const {
        return _state;
    }

    [[nodiscard]] Matrix2 massMatrix(JointVector2 position) const {
        const double lc1 = _parameters.link1Length * 0.5;
        const double lc2 = _parameters.link2Length * 0.5;
        const double c2 = std::cos(position.second);
        const double m11 = _parameters.link1Inertia + _parameters.link2Inertia +
                           _parameters.link1Mass * lc1 * lc1 +
                           _parameters.link2Mass *
                               (_parameters.link1Length * _parameters.link1Length + lc2 * lc2 +
                                2.0 * _parameters.link1Length * lc2 * c2);
        const double m12 = _parameters.link2Inertia +
                           _parameters.link2Mass * (lc2 * lc2 + _parameters.link1Length * lc2 * c2);
        const double m22 = _parameters.link2Inertia + _parameters.link2Mass * lc2 * lc2;
        return Matrix2{.m11 = m11, .m12 = m12, .m21 = m12, .m22 = m22};
    }

    [[nodiscard]] JointVector2 coriolisVector(JointVector2 position, JointVector2 velocity) const {
        const double lc2 = _parameters.link2Length * 0.5;
        const double h = -_parameters.link2Mass * _parameters.link1Length * lc2 * std::sin(position.second);
        return JointVector2{
            .first = h * (2.0 * velocity.first * velocity.second + velocity.second * velocity.second),
            .second = -h * velocity.first * velocity.first,
        };
    }

    [[nodiscard]] JointVector2 gravityTorque(JointVector2 position) const {
        const double lc1 = _parameters.link1Length * 0.5;
        const double lc2 = _parameters.link2Length * 0.5;
        return JointVector2{
            .first = (_parameters.link1Mass * lc1 + _parameters.link2Mass * _parameters.link1Length) *
                         _parameters.gravity * std::cos(position.first) +
                     _parameters.link2Mass * lc2 * _parameters.gravity *
                         std::cos(position.first + position.second),
            .second = _parameters.link2Mass * lc2 * _parameters.gravity *
                      std::cos(position.first + position.second),
        };
    }

    [[nodiscard]] JointVector2 computedTorque(JointVector2 desiredPosition,
                                              JointVector2 desiredVelocity,
                                              JointVector2 desiredAcceleration,
                                              double kp,
                                              double kd) const {
        const JointVector2 feedbackAcceleration{
            .first = desiredAcceleration.first + kd * (desiredVelocity.first - _state.velocity.first) +
                     kp * (desiredPosition.first - _state.position.first),
            .second = desiredAcceleration.second + kd * (desiredVelocity.second - _state.velocity.second) +
                      kp * (desiredPosition.second - _state.position.second),
        };

        const Matrix2 mass = massMatrix(_state.position);
        const JointVector2 coriolis = coriolisVector(_state.position, _state.velocity);
        const JointVector2 gravityVector = gravityTorque(_state.position);
        return JointVector2{
            .first = mass.m11 * feedbackAcceleration.first + mass.m12 * feedbackAcceleration.second +
                     coriolis.first + gravityVector.first,
            .second = mass.m21 * feedbackAcceleration.first + mass.m22 * feedbackAcceleration.second +
                      coriolis.second + gravityVector.second,
        };
    }

private:
    [[nodiscard]] JointVector2 solveMassMatrix(Matrix2 matrix, JointVector2 rhs) const {
        const double determinant = matrix.m11 * matrix.m22 - matrix.m12 * matrix.m21;
        if (std::abs(determinant) < 1e-9) {
            throw std::runtime_error("TwoLinkArm mass matrix is singular");
        }
        return JointVector2{
            .first = (rhs.first * matrix.m22 - matrix.m12 * rhs.second) / determinant,
            .second = (matrix.m11 * rhs.second - rhs.first * matrix.m21) / determinant,
        };
    }

    void validateParameters() const {
        if (_parameters.link1Length <= 0.0 || _parameters.link2Length <= 0.0 ||
            _parameters.link1Mass <= 0.0 || _parameters.link2Mass <= 0.0 ||
            _parameters.link1Inertia <= 0.0 || _parameters.link2Inertia <= 0.0 ||
            _parameters.gravity < 0.0 || _parameters.jointDamping < 0.0 ||
            _parameters.torqueLimit <= 0.0) {
            throw std::invalid_argument("TwoLinkArm parameters must be valid");
        }
    }

    TwoLinkParameters _parameters{};
    TwoLinkState _state{};
};

} // namespace motor_control::robot
