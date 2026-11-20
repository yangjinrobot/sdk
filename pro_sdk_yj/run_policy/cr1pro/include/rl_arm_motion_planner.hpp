#pragma once

#include "common_types.h"
#include "udp_node.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace pro_policy {

// Arm planning follows RLStandPolicyRunner: presets use a 2.4 s quintic
// interpolation; UDP targets use joint limits and a velocity-limited ramp.
class RLArmMotionPlanner {
    types::VecXf current_ = types::VecXf::Zero(14);
    types::VecXf start_ = types::VecXf::Zero(14);
    types::VecXf goal_ = types::VecXf::Zero(14);
    types::VecXf udp_output_ = types::VecXf::Zero(14);
    std::array<bool, 6> previous_presets_{};
    uint32_t previous_sequence_ = 0;
    float elapsed_ = 0.f, start_time_ = 0.f, duration_ = 2.4f;
    bool initialized_ = false, interpolating_ = false, udp_active_ = false;
    std::array<types::VecXf, 6> preset_targets_;

public:
    // Per-runner overrides survive Reset; other users retain their presets.
    void SetPresetTargets(int preset, const types::Vec7f& left, const types::Vec7f& right);

    void Reset(const types::Vec7f& left, const types::Vec7f& right, bool initialized);

    void SetDuration(float duration);

    const types::VecXf& Update(const types::RobotBasicState& state,
                              const types::UserCommand& command, float dt, UdpNode* node);
};
}  // namespace pro_policy
