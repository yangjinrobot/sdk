#pragma once

#include "policy_runner_base.hpp"

class TangxiaPolicyRunner : public PolicyRunnerBase {
public:
    TangxiaPolicyRunner(const std::string& name, RobotName robot)
        : PolicyRunnerBase(name, robot) {}
};
