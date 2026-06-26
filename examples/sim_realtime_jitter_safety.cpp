#include "motor_control/system/SafetyStateMachine.hpp"
#include "motor_control/utils/CsvLogger.hpp"

#include <cmath>
#include <iostream>

int main() {
    using motor_control::system::SafetyEvent;
    using motor_control::system::SafetyState;
    using motor_control::system::SafetyStateMachine;
    using motor_control::utils::CsvLogger;

    SafetyStateMachine safety;
    static_cast<void>(safety.handle(SafetyEvent::SelfCheckPassed));
    static_cast<void>(safety.handle(SafetyEvent::EnableCommand));

    CsvLogger logger("outputs/realtime_jitter_safety.csv",
                     {"time_s", "dt_s", "jitter_s", "command_current_a", "measured_current_a",
                      "is_enabled", "is_fault", "watchdog_miss"});

    constexpr double nominalDt = 0.001;
    constexpr double duration = 1.5;
    double time = 0.0;

    while (time <= duration) {
        const double jitter = 0.00018 * std::sin(2.0 * 3.14159265358979323846 * 17.0 * time);
        const double dt = nominalDt + jitter;
        const bool watchdogMiss = dt > 0.00115;
        const double commandCurrent = time > 0.55 ? 9.0 : 3.0;
        const double measuredCurrent = commandCurrent + 0.4 * std::sin(80.0 * time);

        if ((watchdogMiss && time > 0.35) || measuredCurrent > 8.0) {
            static_cast<void>(safety.handle(SafetyEvent::FaultDetected));
        }
        if (time > 1.1 && safety.state() == SafetyState::Fault) {
            static_cast<void>(safety.handle(SafetyEvent::FaultCleared));
        }

        logger.writeRow({time,
                         dt,
                         jitter,
                         commandCurrent,
                         measuredCurrent,
                         safety.state() == SafetyState::Enabled ? 1.0 : 0.0,
                         safety.state() == SafetyState::Fault ? 1.0 : 0.0,
                         watchdogMiss ? 1.0 : 0.0});

        time += dt;
    }

    std::cout << "Realtime jitter safety simulation finished.\n";
    std::cout << "CSV: outputs/realtime_jitter_safety.csv\n";
    return 0;
}
