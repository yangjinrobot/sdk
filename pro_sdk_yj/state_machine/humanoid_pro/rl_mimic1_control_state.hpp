#ifndef HUMANOID_PRO_RL_MIMIC1_CONTROL_STATE_HPP_
#define HUMANOID_PRO_RL_MIMIC1_CONTROL_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "mimic1_policy_runner.h"

namespace humanoid_pro {
// Dance policy 0x11018: only entered from AmpWalkingControl and only leaves back
// to AmpWalkingControl (plus the shared damping exits).
class RLMimic1ControlState : public CopiedPolicyControlState {
private:
    static std::shared_ptr<PolicyRunnerBase> MakePolicy(RobotName robot_name) {
        auto policy = std::make_shared<Mimic1PolicyRunner>("mimic1", robot_name);
        policy->SetCmdMaxVel(Vec3f(1.0f, 1.0f, 1.0f));
        return policy;
    }
public:
    RLMimic1ControlState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr, MakePolicy(robot_name),
                                   RLControlMimic1Mode, {AmpWalkingControl}) {}
};
}  // namespace humanoid_pro

#endif
