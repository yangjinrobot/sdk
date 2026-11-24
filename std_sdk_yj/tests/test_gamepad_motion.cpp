#include "gamepad_motion_command.hpp"
#include <stdexcept>

using namespace types;
using interface::GamepadMotionTarget;

static void Check(bool ok) {
    if (!ok) throw std::runtime_error("gamepad motion routing failed");
}

int main() {
    static_assert(Restand == 0x1100B && Run == 0x1100C);
    static_assert(Stand2SitMimic == 0x1100D && Sit2StandMimic == 0x1100E);
    static_assert(DunXia == 0x1100F && ZhanLi == 0x11010);
    static_assert(StandElbowMinus04 == 0x11011 && StandElbowPlus12 == 0x11012);
    static_assert(Run1 == 0x11014 && uint32_t(kRun1) == uint32_t(Run1));
    static_assert(Run2 == 0x11015 && uint32_t(kRun2) == uint32_t(Run2));
    static_assert(Run3 == 0x11016 && uint32_t(kRun3) == uint32_t(Run3));
    static_assert(Run4 == 0x11017 && uint32_t(kRun4) == uint32_t(Run4));
    Check(!IsDisabledRunMotionState(Run));
    Check(IsDisabledRunMotionState(Run1));
    Check(IsDisabledRunMotionState(Run2));
    Check(IsDisabledRunMotionState(Run3));
    Check(IsDisabledRunMotionState(Run4));
    for (uint32_t source : {uint32_t(RLControlMode), uint32_t(Run),
                            uint32_t(Restand),
                            uint32_t(Sit2Stand), uint32_t(ZhanLi)}) {
        Check(GamepadMotionTarget(CR1STANDARD, source, source, false, false, false, false,
                                  true) == JointDamping);
    }
    for (uint32_t source : {uint32_t(WaitingForStand), uint32_t(JointDamping), uint32_t(Stand2Sit),
                            uint32_t(DunXia), uint32_t(ZeroPos), uint32_t(SuspendedStand)})
        Check(GamepadMotionTarget(CR1STANDARD, source, source, false, false, false, false,
                                  false) == source);
    Check(GamepadMotionTarget(CR1STANDARD, Run, Run, false, false, true, false,
                              false) == RLControlMode);
    Check(GamepadMotionTarget(CR1STANDARD, Run, Run, false, false, false, true,
                              false) == Run);
    Check(IsStandElbowCommand(StandElbowMinus04) && IsStandElbowCommand(StandElbowPlus12));
    Check(!IsStandElbowCommand(Restand));
    Check(CanAcceptStandElbowCommand(Restand, Restand, 0));
    Check(CanAcceptStandElbowCommand(Restand, StandElbowMinus04, 0));
    Check(!CanAcceptStandElbowCommand(Restand, JointDamping, 0));
    Check(!CanAcceptStandElbowCommand(Restand, ZeroPos, 0));
    Check(!CanAcceptStandElbowCommand(Restand, Restand, 1));
    for (uint32_t source : {uint32_t(RLControlMode), uint32_t(Run), uint32_t(DunXia),
                            uint32_t(ZhanLi), uint32_t(Sit2StandMimic), uint32_t(WaitingForStand)})
        Check(!CanAcceptStandElbowCommand(source, Restand, 0));
    Check(NormalizeMotionStateCommand(0x2000B) == Run);
    Check(NormalizeMotionStateCommand(0x2000C) == Restand);
    Check(NormalizeMotionStateCommand(0x2000D) == Stand2SitMimic);
    Check(NormalizeMotionStateCommand(0x2000E) == Sit2StandMimic);
    Check(NormalizeMotionStateCommand(0x18) == Stand2SitMimic);
    Check(NormalizeMotionStateCommand(0x19) == Sit2StandMimic);
    for (uint32_t code = 0x1100B; code <= 0x1101E; ++code)
        Check(NormalizeMotionStateCommand(code) == code);
    Check(GamepadMotionTarget(CR1STANDARD, DunXia, DunXia, false, true, false, false, false) == ZhanLi);
    Check(GamepadMotionTarget(CR1STANDARD, ZhanLi, ZhanLi, false, false, true, false, false) == RLControlMode);
    Check(GamepadMotionTarget(CR1STANDARD, ZhanLi, ZhanLi, false, false, false, true, false) == Run);
    Check(GamepadMotionTarget(CR1STANDARD, DunXia, DunXia, true, true, true, true, true) == JointDamping);
    for (uint32_t state : {uint32_t(RLControlMode), uint32_t(Run)}) {
        Check(GamepadMotionTarget(CR1STANDARD, state, state, true, false, false, false, false) == Stand2Sit);
        Check(GamepadMotionTarget(CR1PRO, state, state, true, false, false, false, false) == state);
    }
    Check(GamepadMotionTarget(CR1STANDARD, Stand2Sit, Stand2Sit, false, true, false, false, false) == Sit2Stand);
    // No press edge (held/released): retain the pending command.
    Check(GamepadMotionTarget(CR1STANDARD, Stand2Sit, Sit2Stand, false, false, false, false, false) == Sit2Stand);
    Check(GamepadMotionTarget(CR1STANDARD, Sit2Stand, Sit2Stand, false, false, true, false, false) == RLControlMode);
    Check(GamepadMotionTarget(CR1STANDARD, Sit2Stand, Sit2Stand, false, false, false, true, false) == Run);
    for (uint32_t state : {uint32_t(WaitingForStand), uint32_t(JointDamping), uint32_t(ZeroPos), uint32_t(SuspendedStand)})
        Check(GamepadMotionTarget(CR1STANDARD, state, state, true, true, true, true, false) == state);
    // Seated state cannot switch directly to walking.
    Check(GamepadMotionTarget(CR1STANDARD, Stand2Sit, Stand2Sit, false, false, true, true, false) == Stand2Sit);
    for (uint32_t state : {uint32_t(RLControlMode), uint32_t(Run),
                           uint32_t(Stand2Sit), uint32_t(Sit2Stand)})
        Check(GamepadMotionTarget(CR1STANDARD, state, state, true, true, true, true, true) == JointDamping);
    std::cout << "PASS: new state codes, gamepad routes, pending commands, robot restrictions, emergency priority\n";
}
