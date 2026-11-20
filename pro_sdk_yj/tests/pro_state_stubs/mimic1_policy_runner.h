#pragma once
#include "policy_runner_base.hpp"
class Mimic1PolicyRunner : public PolicyRunnerBase {
public:
    Mimic1PolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {}
};
