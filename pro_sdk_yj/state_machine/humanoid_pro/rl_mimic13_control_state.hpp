#ifndef HUMANOID_PRO_RL_MIMIC13_CONTROL_STATE_HPP_
#define HUMANOID_PRO_RL_MIMIC13_CONTROL_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "mimic13_policy_runner.h"

namespace humanoid_pro {
class RLMimic13ControlState : public CopiedPolicyControlState {
private:
    static std::shared_ptr<PolicyRunnerBase> MakePolicy(RobotName robot_name) {
        auto policy = std::make_shared<Mimic13PolicyRunner>("mimic13", robot_name);
        policy->SetCmdMaxVel(Vec3f(1.0f, 1.0f, 1.0f));
        return policy;
    }
public:
    RLMimic13ControlState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr, MakePolicy(robot_name),
                                   RLControlMimic13Mode, {AmpWalkingControl}, std::nullopt,
                                   RLControlMimic12Mode,
                                   Mimic13PolicyRunner::kMotionSteps) {}
};
}  // namespace humanoid_pro

#endif
