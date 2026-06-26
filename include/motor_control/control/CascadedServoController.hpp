#pragma once

#include "motor_control/control/PidController.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace motor_control::control {

struct CascadedServoConfig {
    PidGains positionGains{30.0, 0.0, 0.0};
    PidGains velocityGains{0.4, 2.0, 0.0};
    PidGains currentGains{3.0, 100.0, 0.0};
    double maxVelocity{20.0};
    double maxCurrent{8.0};
    double maxVoltage{24.0};
};

struct CascadedServoCommand {
    double targetPosition{0.0};
    double velocityFeedforward{0.0};
    double currentFeedforward{0.0};
};

struct CascadedServoFeedback {
    double position{0.0};
    double velocity{0.0};
    double current{0.0};
};

struct CascadedServoOutput {
    double velocitySetpoint{0.0};
    double currentSetpoint{0.0};
    double voltageCommand{0.0};
};

class CascadedServoController {
public:
    explicit CascadedServoController(CascadedServoConfig config = {})
        : _config(config),
          _positionLoop(config.positionGains,
                        PidLimits{.minOutput = -config.maxVelocity,
                                  .maxOutput = config.maxVelocity,
                                  .minIntegrator = -config.maxVelocity,
                                  .maxIntegrator = config.maxVelocity}),
          _velocityLoop(config.velocityGains,
                        PidLimits{.minOutput = -config.maxCurrent,
                                  .maxOutput = config.maxCurrent,
                                  .minIntegrator = -config.maxCurrent,
                                  .maxIntegrator = config.maxCurrent}),
          _currentLoop(config.currentGains,
                       PidLimits{.minOutput = -config.maxVoltage,
                                 .maxOutput = config.maxVoltage,
                                 .minIntegrator = -config.maxVoltage,
                                 .maxIntegrator = config.maxVoltage}) {
        validateConfig();
    }

    void reset() {
        _positionLoop.reset();
        _velocityLoop.reset();
        _currentLoop.reset();
    }

    [[nodiscard]] CascadedServoOutput update(const CascadedServoCommand& command,
                                             const CascadedServoFeedback& feedback,
                                             double dt) {
        if (dt <= 0.0 || !std::isfinite(dt)) {
            throw std::invalid_argument("Cascaded servo update requires positive finite dt");
        }

        CascadedServoOutput output;
        output.velocitySetpoint =
            clamp(_positionLoop.update(command.targetPosition, feedback.position, dt) +
                      command.velocityFeedforward,
                  -_config.maxVelocity,
                  _config.maxVelocity);

        output.currentSetpoint =
            clamp(_velocityLoop.update(output.velocitySetpoint, feedback.velocity, dt) +
                      command.currentFeedforward,
                  -_config.maxCurrent,
                  _config.maxCurrent);

        output.voltageCommand =
            clamp(_currentLoop.update(output.currentSetpoint, feedback.current, dt),
                  -_config.maxVoltage,
                  _config.maxVoltage);

        return output;
    }

private:
    static double clamp(double value, double lower, double upper) {
        return std::clamp(value, lower, upper);
    }

    void validateConfig() const {
        if (_config.maxVelocity <= 0.0 || _config.maxCurrent <= 0.0 || _config.maxVoltage <= 0.0) {
            throw std::invalid_argument("Cascaded servo limits must be positive");
        }
    }

    CascadedServoConfig _config{};
    PidController _positionLoop;
    PidController _velocityLoop;
    PidController _currentLoop;
};

} // namespace motor_control::control
