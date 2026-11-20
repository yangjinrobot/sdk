#pragma once
#include "policy_runner_base.hpp"
class Mimic12PolicyRunner : public PolicyRunnerBase {
public:
    Mimic12PolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {}
};
