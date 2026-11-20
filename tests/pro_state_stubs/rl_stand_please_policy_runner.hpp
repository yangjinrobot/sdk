#pragma once
#include "rl_stand_policy_runner.hpp"
class RLStandPleasePolicyRunner : public RLStandPolicyRunner {
public:
    RLStandPleasePolicyRunner(const std::string& name, RobotName robot) : RLStandPolicyRunner(name, robot) {}
    void SetLeftArmMotionEnabled(bool) {}
    void PreservePolicyOnExit(bool) {}
    void RequestArmLowering() { ProPolicyFaults::salute_lowering_requested = true; }
    bool IsArmLoweringComplete() const { return ProPolicyFaults::salute_lowering_complete.load(); }
};
