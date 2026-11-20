#pragma once
#include "rl_stand_policy_runner.hpp"
class RLStandTalkingPolicyRunner : public RLStandPolicyRunner {
public:
    RLStandTalkingPolicyRunner(const std::string& name, RobotName robot) : RLStandPolicyRunner(name, robot) {}
    void SetLeftArmMotionEnabled(bool) {}
    void PreservePolicyOnExit(bool) {}
    void RequestArmLowering() { ProPolicyFaults::salute_lowering_requested = true; }
    bool IsArmLoweringComplete() const { return ProPolicyFaults::salute_lowering_complete.load(); }
};
