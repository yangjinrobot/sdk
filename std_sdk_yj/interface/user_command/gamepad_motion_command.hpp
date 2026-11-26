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
    if ((state == RobotMotionState::AmpControl_NiRenWalk_Mode ||
         state == RobotMotionState::AmpControl_GunShootWalk_Mode ||
         state == RobotMotionState::AmpControl_Walk2Run_Mode) && sit)
        return RobotMotionState::MimicControl_ChairDown_Mode;
    if (state == RobotMotionState::MimicControl_Kneel_Mode && stand)
        return RobotMotionState::MimicControl_Kneel_StandUp_Mode;
    if (state == RobotMotionState::MimicControl_ChairDown_Mode && stand)
        return RobotMotionState::MimicControl_ChairUp_Mode;
    if (state == RobotMotionState::MimicControl_Kneel_StandUp_Mode || state == RobotMotionState::RLControl_Stand_Mode ||
        state == RobotMotionState::MimicControl_ChairUp_Mode || state == RobotMotionState::AmpControl_GunShootWalk_Mode ||
        state == RobotMotionState::AmpControl_NiRenWalk_Mode ||
        state == RobotMotionState::AmpControl_Walk2Run_Mode) {
        if (rl) return RobotMotionState::AmpControl_NiRenWalk_Mode;
        if (niren) return RobotMotionState::AmpControl_Walk2Run_Mode;
    }
    return pending;
}
} // namespace interface
