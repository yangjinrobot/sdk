#pragma once
#include "policy_runner_base.hpp"
class AmpPolicyRunner : public PolicyRunnerBase {
public:
    AmpPolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {}
};
