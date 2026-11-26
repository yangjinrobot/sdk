find_program(CR1_TEST_PYTHON NAMES python3)
if(NOT CR1_TEST_PYTHON)
    message(FATAL_ERROR "Python 3 is required for the CR1 offline tests")
endif()

set(policy_dir "${PROJECT_SOURCE_DIR}/run_policy/cr1")
set(onnx_dir "${PROJECT_SOURCE_DIR}/third_party/onnxruntime/x86")
find_package(Threads REQUIRED)

add_executable(test_rlwalk_policy
    test_rlwalk_policy.cpp
    "${policy_dir}/src/amp_walk_policy_runner.cpp"
)
target_compile_features(test_rlwalk_policy PRIVATE cxx_std_17)
target_compile_definitions(test_rlwalk_policy PRIVATE EIGEN_DONT_ALIGN_STATICALLY)
target_compile_options(test_rlwalk_policy PRIVATE -UNDEBUG)
target_include_directories(test_rlwalk_policy PRIVATE
    "${policy_dir}/include"
    "${PROJECT_SOURCE_DIR}/types"
    "${PROJECT_SOURCE_DIR}/utils"
    "${PROJECT_SOURCE_DIR}/state_machine"
    "${PROJECT_SOURCE_DIR}/third_party/eigen"
    "${onnx_dir}/include"
)
target_link_libraries(test_rlwalk_policy PRIVATE "${onnx_dir}/lib/libonnxruntime.so" Threads::Threads)
add_test(NAME cr1_rlwalk COMMAND test_rlwalk_policy "${policy_dir}/policy/amp/1118/nirenwalk.onnx")

add_executable(test_amp_control_gun_shoot_walk_mode_policy
    test_amp_control_gun_shoot_walk_mode_policy.cpp
    "${policy_dir}/src/amp_control_gun_shoot_walk_mode_policy_runner.cpp"
)
target_compile_features(test_amp_control_gun_shoot_walk_mode_policy PRIVATE cxx_std_17)
target_compile_definitions(test_amp_control_gun_shoot_walk_mode_policy PRIVATE EIGEN_DONT_ALIGN_STATICALLY)
target_compile_options(test_amp_control_gun_shoot_walk_mode_policy PRIVATE -UNDEBUG)
target_include_directories(test_amp_control_gun_shoot_walk_mode_policy PRIVATE
    "${policy_dir}/include"
    "${PROJECT_SOURCE_DIR}/types"
    "${PROJECT_SOURCE_DIR}/utils"
    "${PROJECT_SOURCE_DIR}/state_machine"
    "${PROJECT_SOURCE_DIR}/third_party/eigen"
    "${onnx_dir}/include"
)
target_link_libraries(test_amp_control_gun_shoot_walk_mode_policy PRIVATE
    "${onnx_dir}/lib/libonnxruntime.so" Threads::Threads)
add_test(NAME cr1_amp_control_gun_shoot_walk
    COMMAND test_amp_control_gun_shoot_walk_mode_policy "${policy_dir}/policy/amp/std_amp_2.onnx")

add_executable(test_amp_control_ni_ren_walk_mode_policy
    test_amp_control_ni_ren_walk_mode_policy.cpp
    "${policy_dir}/src/amp_control_ni_ren_walk_mode_policy_runner.cpp"
)
target_compile_features(test_amp_control_ni_ren_walk_mode_policy PRIVATE cxx_std_17)
target_compile_definitions(test_amp_control_ni_ren_walk_mode_policy PRIVATE EIGEN_DONT_ALIGN_STATICALLY)
target_compile_options(test_amp_control_ni_ren_walk_mode_policy PRIVATE -UNDEBUG)
target_include_directories(test_amp_control_ni_ren_walk_mode_policy PRIVATE
    "${policy_dir}/include"
    "${PROJECT_SOURCE_DIR}/types"
    "${PROJECT_SOURCE_DIR}/utils"
    "${PROJECT_SOURCE_DIR}/state_machine"
    "${PROJECT_SOURCE_DIR}/third_party/eigen"
    "${onnx_dir}/include"
)
target_link_libraries(test_amp_control_ni_ren_walk_mode_policy PRIVATE
    "${onnx_dir}/lib/libonnxruntime.so" Threads::Threads)
add_test(NAME cr1_amp_control_ni_ren_walk
    COMMAND test_amp_control_ni_ren_walk_mode_policy
            "${policy_dir}/policy/amp/nirenxingzou_fuzai/policy_15500.onnx")

add_executable(test_mjlab_control_walk2run_mode test_mjlab_control_walk2run_mode.cpp "${policy_dir}/src/mjlab_control_walk2run_mode_policy_runner.cpp")
target_compile_features(test_mjlab_control_walk2run_mode PRIVATE cxx_std_17)
target_compile_definitions(test_mjlab_control_walk2run_mode PRIVATE EIGEN_DONT_ALIGN_STATICALLY MOTION_MASTER_MUJOCO_LOCKSTEP)
target_compile_options(test_mjlab_control_walk2run_mode PRIVATE -UNDEBUG)
target_include_directories(test_mjlab_control_walk2run_mode PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/mimic_stubs"
    "${policy_dir}/include"
    "${PROJECT_SOURCE_DIR}/types"
    "${PROJECT_SOURCE_DIR}/utils"
    "${PROJECT_SOURCE_DIR}/state_machine"
    "${PROJECT_SOURCE_DIR}/third_party/eigen"
    "${onnx_dir}/include"
)
target_link_libraries(test_mjlab_control_walk2run_mode PRIVATE "${onnx_dir}/lib/libonnxruntime.so" Threads::Threads)
add_test(NAME cr1_mjlab_control_walk2run COMMAND test_mjlab_control_walk2run_mode "${policy_dir}/policy/amp/mjlab/model_19999.onnx")

add_test(NAME cr1_rl_control_stand_mode
    COMMAND "${CR1_TEST_PYTHON}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test_restand.py")
add_test(NAME cr1_mimic_control_chair
    COMMAND "${CR1_TEST_PYTHON}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test_sit_stand.py")
add_test(NAME cr1_mimic_control_kneel
    COMMAND "${CR1_TEST_PYTHON}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test_squat.py")
add_test(NAME cr1_amp_control_walk2run_mode_deceleration
    COMMAND "${CR1_TEST_PYTHON}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test_nirenwalk_deceleration.py")
