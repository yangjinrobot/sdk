#ifndef HUMANOID_STANDARD_MOTION_STATE_HPP_
#define HUMANOID_STANDARD_MOTION_STATE_HPP_

#include "custom_types.h"

namespace humanoid {
// CR1STANDARD SDK wire values. The PRO command whitelist is defined separately.
enum StandardMotionState : uint32_t {
    WaitingForStand = 0x0,
    JointDamping = 0x2,
    RLControlMode = 0x11,
    ZeroPos = 0x20001,
    ZeroCalibration = 0x20004,
    SuspendedStand = 0x20006,
};

inline constexpr auto kSuspendedStand = static_cast<types::StateName>(SuspendedStand);
inline constexpr auto kZeroCalibration = static_cast<types::StateName>(ZeroCalibration);
}  // namespace humanoid

#endif
