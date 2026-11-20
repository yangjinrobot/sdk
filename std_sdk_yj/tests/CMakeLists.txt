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

foreach(test_name restand sit_stand nirenwalk_deceleration)
    add_test(NAME "cr1_${test_name}"
        COMMAND "${CR1_TEST_PYTHON}" -B "${CMAKE_CURRENT_SOURCE_DIR}/test_${test_name}.py")
endforeach()
