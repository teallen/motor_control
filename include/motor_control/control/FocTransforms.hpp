#pragma once

#include <cmath>

namespace motor_control::control {

constexpr double PI = 3.141592653589793238462643383279502884;
constexpr double SQRT_THREE = 1.732050807568877293527446341505872367;

struct Abc {
    double a{0.0};
    double b{0.0};
    double c{0.0};
};

struct AlphaBeta {
    double alpha{0.0};
    double beta{0.0};
};

struct Dq {
    double d{0.0};
    double q{0.0};
};

[[nodiscard]] inline double normalizeAngle(double angle) {
    const double twoPi = 2.0 * PI;
    double normalized = std::fmod(angle, twoPi);
    if (normalized < 0.0) {
        normalized += twoPi;
    }
    return normalized;
}

[[nodiscard]] inline AlphaBeta clarkeTransform(const Abc& phase) {
    return AlphaBeta{
        .alpha = (2.0 / 3.0) * (phase.a - 0.5 * phase.b - 0.5 * phase.c),
        .beta = (2.0 / 3.0) * (SQRT_THREE * 0.5) * (phase.b - phase.c),
    };
}

[[nodiscard]] inline Abc inverseClarkeTransform(const AlphaBeta& stationary) {
    return Abc{
        .a = stationary.alpha,
        .b = -0.5 * stationary.alpha + (SQRT_THREE * 0.5) * stationary.beta,
        .c = -0.5 * stationary.alpha - (SQRT_THREE * 0.5) * stationary.beta,
    };
}

[[nodiscard]] inline Dq parkTransform(const AlphaBeta& stationary, double electricalAngle) {
    const double angle = normalizeAngle(electricalAngle);
    const double sinTheta = std::sin(angle);
    const double cosTheta = std::cos(angle);

    return Dq{
        .d = stationary.alpha * cosTheta + stationary.beta * sinTheta,
        .q = -stationary.alpha * sinTheta + stationary.beta * cosTheta,
    };
}

[[nodiscard]] inline AlphaBeta inverseParkTransform(const Dq& rotating, double electricalAngle) {
    const double angle = normalizeAngle(electricalAngle);
    const double sinTheta = std::sin(angle);
    const double cosTheta = std::cos(angle);

    return AlphaBeta{
        .alpha = rotating.d * cosTheta - rotating.q * sinTheta,
        .beta = rotating.d * sinTheta + rotating.q * cosTheta,
    };
}

} // namespace motor_control::control
