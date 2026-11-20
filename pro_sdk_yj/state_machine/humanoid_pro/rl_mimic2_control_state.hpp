#ifndef HUMANOID_PRO_RL_MIMIC2_CONTROL_STATE_HPP_
#define HUMANOID_PRO_RL_MIMIC2_CONTROL_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "mimic2_policy_runner.h"

namespace humanoid_pro {
// Dance policy 0x11019: only entered from AmpWalkingControl and only leaves back
// to AmpWalkingControl (plus the shared damping exits).
class RLMimic2ControlState : public CopiedPolicyControlState {
private:
    static std::shared_ptr<PolicyRunnerBase> MakePolicy(RobotName robot_name) {
        auto policy = std::make_shared<Mimic2PolicyRunner>("mimic2", robot_name);
        policy->SetCmdMaxVel(Vec3f(1.0f, 1.0f, 1.0f));
        return policy;
    }
public:
    RLMimic2ControlState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr, MakePolicy(robot_name),
                                   RLControlMimic2Mode, {AmpWalkingControl}) {}
};
}  // namespace humanoid_pro

#endif
