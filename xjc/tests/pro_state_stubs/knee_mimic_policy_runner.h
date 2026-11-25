#pragma once
#include "policy_runner_base.hpp"
class Stand2KneePolicyRunner : public PolicyRunnerBase {
public:
    Stand2KneePolicyRunner(const std::string& name, RobotName robot,
                          const std::string& = "", UdpNode* = nullptr) : PolicyRunnerBase(name, robot) {}
};
class Knee2StandPolicyRunner : public PolicyRunnerBase {
public:
    Knee2StandPolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {}
};
