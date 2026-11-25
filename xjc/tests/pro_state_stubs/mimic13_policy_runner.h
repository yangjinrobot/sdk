#pragma once
#include "policy_runner_base.hpp"
class Mimic13PolicyRunner : public PolicyRunnerBase {
public:
    Mimic13PolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {}
};
