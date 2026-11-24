#pragma once

#include "custom_types.h"

namespace interface {
// Inputs are press edges; emergency stop is a level and always takes priority.
inline uint32_t GamepadMotionTarget(types::RobotName robot, uint32_t state,
                                    uint32_t pending, bool sit, bool stand,
                                    bool rl, bool niren, bool emergency) {
    using namespace types;
    if (emergency) return RobotMotionState::JointDamping;
    if (robot != RobotName::CR1STANDARD) return pending;
    if ((state == RobotMotionState::RLControlMode ||
         state == RobotMotionState::Run) && sit)
        return RobotMotionState::Stand2Sit;
    if (state == RobotMotionState::DunXia && stand)
        return RobotMotionState::ZhanLi;
    if (state == RobotMotionState::Stand2Sit && stand)
        return RobotMotionState::Sit2Stand;
    if (state == RobotMotionState::ZhanLi || state == RobotMotionState::Restand ||
        state == RobotMotionState::Sit2Stand || state == RobotMotionState::RLControlMode ||
        state == RobotMotionState::Run) {
        if (rl) return RobotMotionState::RLControlMode;
        if (niren) return RobotMotionState::Run;
    }
    return pending;
}
} // namespace interface
