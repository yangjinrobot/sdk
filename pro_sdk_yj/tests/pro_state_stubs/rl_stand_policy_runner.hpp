#pragma once
#include "policy_runner_base.hpp"
class RLStandPolicyRunner : public PolicyRunnerBase {
public:
    RLStandPolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) { decimation_ = 10; }
};
