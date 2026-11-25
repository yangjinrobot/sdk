#pragma once
#include "policy_runner_base.hpp"
class AmpWalkingPolicyRunner : public PolicyRunnerBase {
public:
    void SetArmReleaseFrom(const Vec7f&, const Vec7f&, double = 2.4) {}
    AmpWalkingPolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) { decimation_ = 20; }
};
