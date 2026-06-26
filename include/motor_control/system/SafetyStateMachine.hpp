#pragma once

#include <string_view>

namespace motor_control::system {

enum class SafetyState {
    Boot,
    Ready,
    Enabled,
    Fault,
};

enum class SafetyEvent {
    SelfCheckPassed,
    EnableCommand,
    DisableCommand,
    FaultDetected,
    FaultCleared,
};

class SafetyStateMachine {
public:
    [[nodiscard]] SafetyState state() const {
        return _state;
    }

    [[nodiscard]] bool handle(SafetyEvent event) {
        switch (_state) {
        case SafetyState::Boot:
            if (event == SafetyEvent::SelfCheckPassed) {
                _state = SafetyState::Ready;
                return true;
            }
            if (event == SafetyEvent::FaultDetected) {
                _state = SafetyState::Fault;
                return true;
            }
            return false;
        case SafetyState::Ready:
            if (event == SafetyEvent::EnableCommand) {
                _state = SafetyState::Enabled;
                return true;
            }
            if (event == SafetyEvent::FaultDetected) {
                _state = SafetyState::Fault;
                return true;
            }
            return false;
        case SafetyState::Enabled:
            if (event == SafetyEvent::DisableCommand) {
                _state = SafetyState::Ready;
                return true;
            }
            if (event == SafetyEvent::FaultDetected) {
                _state = SafetyState::Fault;
                return true;
            }
            return false;
        case SafetyState::Fault:
            if (event == SafetyEvent::FaultCleared) {
                _state = SafetyState::Ready;
                return true;
            }
            return false;
        }
        return false;
    }

private:
    SafetyState _state{SafetyState::Boot};
};

[[nodiscard]] inline std::string_view toString(SafetyState state) {
    switch (state) {
    case SafetyState::Boot:
        return "boot";
    case SafetyState::Ready:
        return "ready";
    case SafetyState::Enabled:
        return "enabled";
    case SafetyState::Fault:
        return "fault";
    }
    return "unknown";
}

} // namespace motor_control::system
