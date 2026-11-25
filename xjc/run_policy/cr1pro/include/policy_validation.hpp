#pragma once

#include "common_types.h"
#include <onnxruntime_cxx_api.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace pro_policy {

inline void ValidateModelSchema(const Ort::Session& session, int observation_dim,
                                int action_dim, const char* input_name,
                                const char* output_name) {
    if (session.GetInputCount() != 1 || session.GetOutputCount() != 1)
        throw std::runtime_error("CR1PRO model must have one input and one output");
    Ort::AllocatorWithDefaultOptions allocator;
    const auto check = [&](bool input, int width, const char* expected_name) {
        auto name = input ? session.GetInputNameAllocated(0, allocator)
                          : session.GetOutputNameAllocated(0, allocator);
        auto type = input ? session.GetInputTypeInfo(0) : session.GetOutputTypeInfo(0);
        auto tensor = type.GetTensorTypeAndShapeInfo();
        const auto shape = tensor.GetShape();
        if (std::string(name.get()) != expected_name ||
            tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            shape.size() != 2 || (shape[0] != 1 && shape[0] != -1) || shape[1] != width)
            throw std::runtime_error(std::string("CR1PRO model ") +
                                     (input ? "input" : "output") + " schema mismatch");
    };
    check(true, observation_dim, input_name);
    check(false, action_dim, output_name);
}

inline void ValidateRobotState(const types::RobotBasicState& state,
                               const types::UserCommand& command) {
    for (const auto* vector : {&state.joint_pos, &state.joint_vel, &state.joint_tau}) {
        if ((vector->size() != 29 && vector->size() != 31) || !vector->allFinite())
            throw std::runtime_error("CR1PRO policy requires finite 29/31 joint state vectors");
    }
    if (!state.base_omega.allFinite() || !state.base_rot_mat.allFinite() ||
        !std::isfinite(command.forward_vel_scale) ||
        !std::isfinite(command.side_vel_scale) || !std::isfinite(command.turnning_vel_scale))
        throw std::runtime_error("CR1PRO policy received non-finite IMU or velocity input");
}

inline void ValidateObservation(const types::VecXf& observation, int expected_dim) {
    if (observation.size() != expected_dim || !observation.allFinite())
        throw std::runtime_error("CR1PRO policy observation dimension/finite check failed");
}

inline types::VecXf ReadAction(const std::vector<Ort::Value>& outputs, int action_dim) {
    if (outputs.size() != 1 || !outputs.front().IsTensor())
        throw std::runtime_error("CR1PRO policy output is not a single tensor");
    const auto info = outputs.front().GetTensorTypeAndShapeInfo();
    if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        info.GetShape() != std::vector<int64_t>{1, action_dim} ||
        info.GetElementCount() != static_cast<size_t>(action_dim))
        throw std::runtime_error("CR1PRO policy action shape/type mismatch");
    types::VecXf action = Eigen::Map<const types::VecXf>(
        outputs.front().GetTensorData<float>(), action_dim);
    if (!action.allFinite()) throw std::runtime_error("CR1PRO policy action is non-finite");
    return action;
}

inline void ValidateRobotAction(const types::RobotAction& action) {
    for (const auto* vector : {&action.goal_joint_pos, &action.goal_joint_vel,
                              &action.kp, &action.kd, &action.tau_ff}) {
        if (vector->size() != 29 || !vector->allFinite())
            throw std::runtime_error("CR1PRO policy produced invalid 29-joint commands");
    }
}

}  // namespace pro_policy
