cmake_minimum_required(VERSION 3.16)

project(MotorControlCourse LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(motor_control INTERFACE)
target_include_directories(motor_control INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

function(add_course_example target_name source_file)
    add_executable(${target_name} ${source_file})
    target_link_libraries(${target_name} PRIVATE motor_control)
    set_target_properties(${target_name} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/examples
    )
endfunction()

add_course_example(sim_dc_position_pid examples/sim_dc_position_pid.cpp)
add_course_example(sim_cascaded_servo examples/sim_cascaded_servo.cpp)
add_course_example(sim_pmsm_foc examples/sim_pmsm_foc.cpp)
add_course_example(sim_encoder_velocity_filter examples/sim_encoder_velocity_filter.cpp)
add_course_example(sim_frequency_response examples/sim_frequency_response.cpp)
add_course_example(sim_disturbance_observer examples/sim_disturbance_observer.cpp)
add_course_example(sim_single_joint_gravity_friction examples/sim_single_joint_gravity_friction.cpp)
add_course_example(sim_joint_impedance examples/sim_joint_impedance.cpp)
add_course_example(sim_two_link_computed_torque examples/sim_two_link_computed_torque.cpp)
add_course_example(sim_realtime_jitter_safety examples/sim_realtime_jitter_safety.cpp)
add_course_example(sim_tuning_suite examples/sim_tuning_suite.cpp)

enable_testing()

add_executable(motor_control_tests tests/test_core.cpp)
target_link_libraries(motor_control_tests PRIVATE motor_control)
set_target_properties(motor_control_tests PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests
)

add_test(NAME motor_control_tests COMMAND motor_control_tests)
