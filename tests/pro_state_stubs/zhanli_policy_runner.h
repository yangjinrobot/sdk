#pragma once

#include "policy_runner_base.hpp"

class ZhanliPolicyRunner : public PolicyRunnerBase {
public:
    ZhanliPolicyRunner(const std::string& name, RobotName robot)
        : PolicyRunnerBase(name, robot) {}
};
