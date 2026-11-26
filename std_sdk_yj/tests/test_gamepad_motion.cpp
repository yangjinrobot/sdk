#include "gamepad_motion_command.hpp"
#include <stdexcept>

using namespace types;
using interface::GamepadMotionTarget;

static void Check(bool ok) {
    if (!ok) throw std::runtime_error("gamepad motion routing failed");
}

int main() {
    static_assert(AmpControl_NiRenWalk_Mode == 0x11);
    static_assert(AmpControl_GunShootWalk_Mode == 0x1100B && RLControl_Stand_Mode == 0x1100C);
    static_assert(AmpControl_Walk2Run_Mode == 0x1100D && MimicControl_ChairDown_Mode == 0x1100E);
    static_assert(MimicControl_ChairUp_Mode == 0x1100F && MimicControl_Kneel_Mode == 0x11010);
    static_assert(MimicControl_Kneel_StandUp_Mode == 0x11011);
    static_assert(StandElbowMinus04 == 0x11012 && StandElbowPlus12 == 0x11013);
    for (uint32_t source : {uint32_t(AmpControl_NiRenWalk_Mode), uint32_t(AmpControl_GunShootWalk_Mode), uint32_t(AmpControl_Walk2Run_Mode),
                            uint32_t(RLControl_Stand_Mode),
                            uint32_t(MimicControl_ChairUp_Mode), uint32_t(MimicControl_Kneel_StandUp_Mode)}) {
        Check(GamepadMotionTarget(CR1STANDARD, source, source, false, false, false, false,
                                  true) == JointDamping);
    }
    for (uint32_t source : {uint32_t(WaitingForStand), uint32_t(JointDamping), uint32_t(MimicControl_ChairDown_Mode),
                            uint32_t(MimicControl_Kneel_Mode), uint32_t(ZeroPos), uint32_t(SuspendedStand)})
        Check(GamepadMotionTarget(CR1STANDARD, source, source, false, false, false, false,
                                  false) == source);
    Check(GamepadMotionTarget(CR1STANDARD, AmpControl_Walk2Run_Mode, AmpControl_Walk2Run_Mode, false, false, true, false,
                              false) == AmpControl_NiRenWalk_Mode);
    Check(GamepadMotionTarget(CR1STANDARD, AmpControl_Walk2Run_Mode, AmpControl_Walk2Run_Mode, false, false, false, true,
                              false) == AmpControl_Walk2Run_Mode);
    Check(IsStandElbowCommand(StandElbowMinus04) && IsStandElbowCommand(StandElbowPlus12));
    Check(!IsStandElbowCommand(RLControl_Stand_Mode));
    Check(RequiresRLControlStandTransition(AmpControl_NiRenWalk_Mode,
                                           AmpControl_GunShootWalk_Mode));
    Check(RequiresRLControlStandTransition(AmpControl_NiRenWalk_Mode,
                                           MimicControl_ChairDown_Mode));
    Check(RequiresRLControlStandTransition(AmpControl_GunShootWalk_Mode,
                                           AmpControl_NiRenWalk_Mode));
    Check(RequiresRLControlStandTransition(AmpControl_GunShootWalk_Mode,
                                           AmpControl_Walk2Run_Mode));
    Check(RequiresRLControlStandTransition(AmpControl_Walk2Run_Mode,
                                           AmpControl_GunShootWalk_Mode));
    Check(RequiresRLControlStandTransition(MimicControl_ChairUp_Mode,
                                           AmpControl_Walk2Run_Mode));
    Check(RequiresRLControlStandTransition(MimicControl_ChairUp_Mode,
                                           AmpControl_NiRenWalk_Mode));
    Check(RequiresRLControlStandTransition(MimicControl_ChairUp_Mode,
                                           AmpControl_GunShootWalk_Mode));
    Check(RequiresRLControlStandTransition(MimicControl_Kneel_StandUp_Mode,
                                           AmpControl_NiRenWalk_Mode));
    Check(RequiresRLControlStandTransition(MimicControl_Kneel_StandUp_Mode,
                                           AmpControl_GunShootWalk_Mode));
    Check(!RequiresRLControlStandTransition(AmpControl_NiRenWalk_Mode,
                                            AmpControl_Walk2Run_Mode));
    Check(CanAcceptStandElbowCommand(RLControl_Stand_Mode, RLControl_Stand_Mode, 0));
    Check(CanAcceptStandElbowCommand(RLControl_Stand_Mode, StandElbowMinus04, 0));
    Check(!CanAcceptStandElbowCommand(RLControl_Stand_Mode, JointDamping, 0));
    Check(!CanAcceptStandElbowCommand(RLControl_Stand_Mode, ZeroPos, 0));
    Check(!CanAcceptStandElbowCommand(RLControl_Stand_Mode, RLControl_Stand_Mode, 1));
    for (uint32_t source : {uint32_t(AmpControl_NiRenWalk_Mode), uint32_t(AmpControl_GunShootWalk_Mode), uint32_t(AmpControl_Walk2Run_Mode), uint32_t(MimicControl_Kneel_Mode),
                            uint32_t(MimicControl_Kneel_StandUp_Mode), uint32_t(MimicControl_ChairUp_Mode), uint32_t(WaitingForStand)})
        Check(!CanAcceptStandElbowCommand(source, RLControl_Stand_Mode, 0));
    Check(NormalizeMotionStateCommand(0x2000B) == AmpControl_Walk2Run_Mode);
    Check(NormalizeMotionStateCommand(0x2000C) == RLControl_Stand_Mode);
    Check(NormalizeMotionStateCommand(0x2000D) == MimicControl_ChairDown_Mode);
    Check(NormalizeMotionStateCommand(0x2000E) == MimicControl_ChairUp_Mode);
    Check(NormalizeMotionStateCommand(0x18) == MimicControl_ChairDown_Mode);
    Check(NormalizeMotionStateCommand(0x19) == MimicControl_ChairUp_Mode);
    for (uint32_t code = 0x1100B; code <= 0x1101E; ++code)
        Check(NormalizeMotionStateCommand(code) == code);
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_Kneel_Mode, MimicControl_Kneel_Mode, false, true, false, false, false) == MimicControl_Kneel_StandUp_Mode);
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_Kneel_StandUp_Mode, MimicControl_Kneel_StandUp_Mode, false, false, true, false, false) == AmpControl_NiRenWalk_Mode);
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_Kneel_StandUp_Mode, MimicControl_Kneel_StandUp_Mode, false, false, false, true, false) == AmpControl_Walk2Run_Mode);
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_Kneel_Mode, MimicControl_Kneel_Mode, true, true, true, true, true) == JointDamping);
    for (uint32_t state : {uint32_t(AmpControl_NiRenWalk_Mode), uint32_t(AmpControl_GunShootWalk_Mode), uint32_t(AmpControl_Walk2Run_Mode)}) {
        Check(GamepadMotionTarget(CR1STANDARD, state, state, true, false, false, false, false) == MimicControl_ChairDown_Mode);
        Check(GamepadMotionTarget(CR1PRO, state, state, true, false, false, false, false) == state);
    }
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_ChairDown_Mode, MimicControl_ChairDown_Mode, false, true, false, false, false) == MimicControl_ChairUp_Mode);
    // No press edge (held/released): retain the pending command.
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_ChairDown_Mode, MimicControl_ChairUp_Mode, false, false, false, false, false) == MimicControl_ChairUp_Mode);
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_ChairUp_Mode, MimicControl_ChairUp_Mode, false, false, true, false, false) == AmpControl_NiRenWalk_Mode);
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_ChairUp_Mode, MimicControl_ChairUp_Mode, false, false, false, true, false) == AmpControl_Walk2Run_Mode);
    for (uint32_t state : {uint32_t(WaitingForStand), uint32_t(JointDamping), uint32_t(ZeroPos), uint32_t(SuspendedStand)})
        Check(GamepadMotionTarget(CR1STANDARD, state, state, true, true, true, true, false) == state);
    // Seated state cannot switch directly to walking.
    Check(GamepadMotionTarget(CR1STANDARD, MimicControl_ChairDown_Mode, MimicControl_ChairDown_Mode, false, false, true, true, false) == MimicControl_ChairDown_Mode);
    for (uint32_t state : {uint32_t(AmpControl_NiRenWalk_Mode), uint32_t(AmpControl_GunShootWalk_Mode), uint32_t(AmpControl_Walk2Run_Mode),
                           uint32_t(MimicControl_ChairDown_Mode), uint32_t(MimicControl_ChairUp_Mode)})
        Check(GamepadMotionTarget(CR1STANDARD, state, state, true, true, true, true, true) == JointDamping);
    std::cout << "PASS: new state codes, gamepad routes, pending commands, robot restrictions, emergency priority\n";
}
