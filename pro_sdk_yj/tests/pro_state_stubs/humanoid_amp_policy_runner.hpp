#pragma once
#include "policy_runner_base.hpp"
class HumanoidAmpPolicyRunner : public PolicyRunnerBase {
public:
    HumanoidAmpPolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) { decimation_ = 20; }
};
