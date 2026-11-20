#pragma once
#include "policy_runner_base.hpp"
class Mimic2PolicyRunner : public PolicyRunnerBase {
public:
    Mimic2PolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {}
};
