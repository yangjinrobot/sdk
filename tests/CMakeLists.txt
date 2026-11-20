cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(PRO_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." ABSOLUTE)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(PRO_TEST_ARCH arm)
else()
    set(PRO_TEST_ARCH x86)
endif()
set(PRO_ORT "${PRO_ROOT}/third_party/onnxruntime/${PRO_TEST_ARCH}")
set(PRO_INCLUDES "${PRO_ROOT}/types" "${PRO_ROOT}/utils" "${PRO_ROOT}/state_machine"
    "${PRO_ROOT}/interface/user_command" "${PRO_ROOT}/run_policy/cr1pro/include"
    "${PRO_ROOT}/third_party/eigen" "${PRO_ROOT}/third_party/toml" "${PRO_ORT}/include")
add_executable(test_pro_commands test_pro_commands.cpp "${PRO_ROOT}/run_policy/cr1pro/src/udp_node.cpp")
target_include_directories(test_pro_commands PRIVATE ${PRO_INCLUDES})
target_compile_definitions(test_pro_commands PRIVATE MOTION_MASTER_COMMANDS_OFFLINE)
target_link_libraries(test_pro_commands PRIVATE pthread)
add_test(NAME pro_commands COMMAND test_pro_commands)

add_executable(test_pro_state_transitions test_pro_state_transitions.cpp
    "${PRO_ROOT}/state_machine/parameters/cr1_pro_control_parameters.cpp"
    "${PRO_ROOT}/state_machine/parameters/cr1_standard_control_parameters.cpp")
target_include_directories(test_pro_state_transitions BEFORE PRIVATE pro_state_stubs)
target_include_directories(test_pro_state_transitions PRIVATE ${PRO_INCLUDES})
target_link_libraries(test_pro_state_transitions PRIVATE pthread)
add_test(NAME pro_state_transitions COMMAND test_pro_state_transitions)

include("${PRO_ROOT}/run_policy/cr1pro/sources.cmake")
add_library(pro_test_policies STATIC ${PRO_POLICY_SOURCES})
target_include_directories(pro_test_policies PUBLIC ${PRO_INCLUDES})
target_compile_definitions(pro_test_policies PUBLIC
    MOTION_MASTER_PRO_POLICY_DIR="${PRO_ROOT}/run_policy/cr1pro/policy")
target_link_libraries(pro_test_policies PUBLIC "${PRO_ORT}/lib/libonnxruntime.so" pthread)

add_executable(test_pro_policies test_pro_policies.cpp)
target_link_libraries(test_pro_policies PRIVATE pro_test_policies)
set_target_properties(test_pro_policies PROPERTIES BUILD_RPATH "${PRO_ORT}/lib")
add_test(NAME pro_policies COMMAND test_pro_policies)

add_executable(test_knee_mimic test_knee_mimic.cpp)
target_link_libraries(test_knee_mimic PRIVATE pro_test_policies)
set_target_properties(test_knee_mimic PROPERTIES BUILD_RPATH "${PRO_ORT}/lib")
add_test(NAME knee_mimic COMMAND test_knee_mimic)

add_executable(test_recovery_policies test_recovery_policies.cpp)
target_link_libraries(test_recovery_policies PRIVATE pro_test_policies)
set_target_properties(test_recovery_policies PROPERTIES BUILD_RPATH "${PRO_ORT}/lib")
add_test(NAME recovery_policies COMMAND test_recovery_policies)

add_executable(test_amp_walking test_amp_walking.cpp)
target_link_libraries(test_amp_walking PRIVATE pro_test_policies)
set_target_properties(test_amp_walking PROPERTIES BUILD_RPATH "${PRO_ORT}/lib")
add_test(NAME amp_walking COMMAND test_amp_walking)

add_executable(test_rl_stand_salute test_rl_stand_salute.cpp)
target_link_libraries(test_rl_stand_salute PRIVATE pro_test_policies)
set_target_properties(test_rl_stand_salute PROPERTIES BUILD_RPATH "${PRO_ORT}/lib")
add_test(NAME rl_stand_salute COMMAND test_rl_stand_salute)

add_executable(test_amp_moe_15dof test_amp_moe_15dof.cpp)
target_link_libraries(test_amp_moe_15dof PRIVATE pro_test_policies)
set_target_properties(test_amp_moe_15dof PROPERTIES BUILD_RPATH "${PRO_ORT}/lib")
add_test(NAME amp_moe_15dof COMMAND test_amp_moe_15dof)

add_executable(test_amp_walking_waq test_amp_walking_waq.cpp)
target_link_libraries(test_amp_walking_waq PRIVATE pro_test_policies)
set_target_properties(test_amp_walking_waq PROPERTIES BUILD_RPATH "${PRO_ORT}/lib")
add_test(NAME amp_walking_waq COMMAND test_amp_walking_waq)

add_executable(test_pro_config test_pro_config.cpp)
target_include_directories(test_pro_config PRIVATE "${PRO_ROOT}/utils" "${PRO_ROOT}/third_party/toml")
add_test(NAME pro_config COMMAND test_pro_config)

add_executable(test_bounded_policy_command test_bounded_policy_command.cpp
    "${PRO_ROOT}/state_machine/parameters/cr1_pro_control_parameters.cpp"
    "${PRO_ROOT}/state_machine/parameters/cr1_standard_control_parameters.cpp")
target_include_directories(test_bounded_policy_command PRIVATE ${PRO_INCLUDES})
add_test(NAME bounded_policy_command COMMAND test_bounded_policy_command)
