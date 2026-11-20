#pragma once
#include "rl_stand_policy_runner.hpp"
class RLStandPlease2PolicyRunner : public RLStandPolicyRunner {
public:
    RLStandPlease2PolicyRunner(const std::string& name, RobotName robot) : RLStandPolicyRunner(name, robot) {}
    void PreservePolicyOnExit(bool) {}
    void RequestArmLowering() { ProPolicyFaults::salute_lowering_requested = true; }
    bool IsArmLoweringComplete() const { return ProPolicyFaults::salute_lowering_complete.load(); }
};
