#ifndef HUMANOID_PRO_RL_MIMIC12_CONTROL_STATE_HPP_
#define HUMANOID_PRO_RL_MIMIC12_CONTROL_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "mimic12_policy_runner.h"

namespace humanoid_pro {
class RLMimic12ControlState : public CopiedPolicyControlState {
private:
    static std::shared_ptr<PolicyRunnerBase> MakePolicy(RobotName robot_name) {
        auto policy = std::make_shared<Mimic12PolicyRunner>("mimic12", robot_name);
        policy->SetCmdMaxVel(Vec3f(1.0f, 1.0f, 1.0f));
        return policy;
    }
public:
    RLMimic12ControlState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr, MakePolicy(robot_name),
                                   MimicControl_ChairDown_Mode, {MimicControl_ChairUp_Mode}) {}
};
}  // namespace humanoid_pro

#endif
