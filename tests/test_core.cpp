#include "motor_control/control/FocTransforms.hpp"
#include "motor_control/control/Filters.hpp"
#include "motor_control/control/ImpedanceController.hpp"
#include "motor_control/control/PidController.hpp"
#include "motor_control/control/Trajectory.hpp"
#include "motor_control/motor/DcMotor.hpp"
#include "motor_control/motor/PmsmDqModel.hpp"
#include "motor_control/robot/SingleJoint.hpp"
#include "motor_control/robot/TwoLinkArm.hpp"
#include "motor_control/system/SafetyStateMachine.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void expectNear(double actual, double expected, double tolerance, const char* message) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << message << ": expected " << expected << ", got " << actual << '\n';
        throw std::runtime_error(message);
    }
}

void expectTrue(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        throw std::runtime_error(message);
    }
}

void testPidOutputLimit() {
    using motor_control::control::PidController;
    using motor_control::control::PidGains;
    using motor_control::control::PidLimits;

    PidController pid(PidGains{.kp = 10.0, .ki = 5.0, .kd = 0.0},
                      PidLimits{.minOutput = -2.0,
                                .maxOutput = 2.0,
                                .minIntegrator = -0.1,
                                .maxIntegrator = 0.1});

    const double output = pid.update(10.0, 0.0, 0.01);
    expectNear(output, 2.0, 1e-12, "PID output should be clamped");
    expectNear(pid.integrator(), 0.1, 1e-12, "PID integrator should be clamped");
}

void testFocTransformsRoundTrip() {
    using motor_control::control::Abc;
    using motor_control::control::clarkeTransform;
    using motor_control::control::inverseClarkeTransform;
    using motor_control::control::inverseParkTransform;
    using motor_control::control::parkTransform;

    const Abc phase{.a = 2.0, .b = -0.75, .c = -1.25};
    const auto stationary = clarkeTransform(phase);
    const auto rotating = parkTransform(stationary, 1.2);
    const auto restoredStationary = inverseParkTransform(rotating, 1.2);
    const auto restoredPhase = inverseClarkeTransform(restoredStationary);

    expectNear(restoredStationary.alpha, stationary.alpha, 1e-12, "Park round trip alpha");
    expectNear(restoredStationary.beta, stationary.beta, 1e-12, "Park round trip beta");
    expectNear(restoredPhase.a, phase.a, 1e-12, "Clarke round trip phase a");
    expectNear(restoredPhase.b, phase.b, 1e-12, "Clarke round trip phase b");
    expectNear(restoredPhase.c, phase.c, 1e-12, "Clarke round trip phase c");
}

void testDcMotorPositiveVoltageAccelerates() {
    using motor_control::motor::DcMotor;

    DcMotor motor;
    for (int index = 0; index < 1000; ++index) {
        motor.update(6.0, 0.0, 0.0005);
    }

    expectTrue(motor.state().current > 0.0, "DC motor current should be positive");
    expectTrue(motor.state().angularVelocity > 0.0, "DC motor velocity should be positive");
    expectTrue(motor.state().angularPosition > 0.0, "DC motor position should be positive");
}

void testPmsmIqProducesPositiveTorque() {
    using motor_control::motor::PmsmDqModel;
    using motor_control::motor::PmsmDqState;

    PmsmDqModel motor;
    motor.reset(PmsmDqState{.dAxisCurrent = 0.0, .qAxisCurrent = 3.0});

    expectTrue(motor.electromagneticTorque() > 0.0, "Positive iq should produce positive torque");
}

void testSmoothStepBoundaryConditions() {
    using motor_control::control::smoothStepTrajectory;

    const auto start = smoothStepTrajectory(0.0, 1.0, 2.0, 0.2, 1.0);
    const auto middle = smoothStepTrajectory(1.5, 1.0, 2.0, 0.2, 1.0);
    const auto end = smoothStepTrajectory(3.0, 1.0, 2.0, 0.2, 1.0);

    expectNear(start.position, 0.2, 1e-12, "Smooth step start position");
    expectTrue(middle.position > 0.2 && middle.position < 1.0, "Smooth step middle position");
    expectNear(end.position, 1.0, 1e-12, "Smooth step end position");
    expectNear(end.velocity, 0.0, 1e-12, "Smooth step end velocity");
}

void testVelocityEstimator() {
    using motor_control::control::VelocityEstimator;

    VelocityEstimator estimator(50.0);
    double velocity = 0.0;
    for (int index = 0; index < 1000; ++index) {
        const double time = static_cast<double>(index) * 0.001;
        velocity = estimator.update(2.0 * time, 0.001);
    }

    expectNear(velocity, 2.0, 0.05, "Velocity estimator should converge");
}

void testSingleJointGravitySign() {
    using motor_control::robot::SingleJoint;

    SingleJoint joint;
    expectTrue(joint.gravityTorque(0.5) > 0.0, "Positive joint position should have positive gravity torque");
}

void testTwoLinkMassMatrixPositive() {
    using motor_control::robot::JointVector2;
    using motor_control::robot::TwoLinkArm;

    TwoLinkArm arm;
    const auto mass = arm.massMatrix(JointVector2{.first = 0.2, .second = -0.4});
    expectTrue(mass.m11 > 0.0, "Two-link mass m11 should be positive");
    expectTrue(mass.m22 > 0.0, "Two-link mass m22 should be positive");
    expectNear(mass.m12, mass.m21, 1e-12, "Two-link mass matrix should be symmetric");
}

void testSafetyStateMachine() {
    using motor_control::system::SafetyEvent;
    using motor_control::system::SafetyState;
    using motor_control::system::SafetyStateMachine;

    SafetyStateMachine machine;
    expectTrue(machine.state() == SafetyState::Boot, "Safety state should start in boot");
    expectTrue(machine.handle(SafetyEvent::SelfCheckPassed), "Self check should transition to ready");
    expectTrue(machine.state() == SafetyState::Ready, "Safety state should be ready");
    expectTrue(machine.handle(SafetyEvent::EnableCommand), "Enable should transition to enabled");
    expectTrue(machine.state() == SafetyState::Enabled, "Safety state should be enabled");
    expectTrue(machine.handle(SafetyEvent::FaultDetected), "Fault should transition to fault");
    expectTrue(machine.state() == SafetyState::Fault, "Safety state should be fault");
    expectTrue(machine.handle(SafetyEvent::FaultCleared), "Fault clear should transition to ready");
    expectTrue(machine.state() == SafetyState::Ready, "Safety state should recover to ready");
}

} // namespace

int main() {
    testPidOutputLimit();
    testFocTransformsRoundTrip();
    testDcMotorPositiveVoltageAccelerates();
    testPmsmIqProducesPositiveTorque();
    testSmoothStepBoundaryConditions();
    testVelocityEstimator();
    testSingleJointGravitySign();
    testTwoLinkMassMatrixPositive();
    testSafetyStateMachine();

    std::cout << "All motor control tests passed.\n";
    return 0;
}
