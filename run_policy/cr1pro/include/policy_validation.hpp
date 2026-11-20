#pragma once

#include "common_types.h"
#include <onnxruntime_cxx_api.h>
#include <stdexcept>
#include <string>
#include <vector>

namespace pro_policy {

void ValidateModelSchema(const Ort::Session& session, int observation_dim,
                                int action_dim, const char* input_name,
                                const char* output_name);

void ValidateRobotState(const types::RobotBasicState& state,
                               const types::UserCommand& command);

void ValidateObservation(const types::VecXf& observation, int expected_dim);

types::VecXf ReadAction(const std::vector<Ort::Value>& outputs, int action_dim);

void ValidateRobotAction(const types::RobotAction& action);

}  // namespace pro_policy
