#pragma once

#include "policy_validation.hpp"

namespace pro_policy {
// The mimic exports include a reference trajectory as well as the actions.
void ValidateCopiedModel(const Ort::Session& session, int observation_dim, bool mimic);

void ValidateMimicOutputs(const std::vector<Ort::Value>& outputs);
}  // namespace pro_policy
