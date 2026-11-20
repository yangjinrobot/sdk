#pragma once
#include "policy_runner_base.hpp"
class Mimic13PolicyRunner : public PolicyRunnerBase {
public:
    static constexpr int kMotionSteps = 360;
    Mimic13PolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {}
};
