#ifndef HUMANOID_PRO_RL_CONTROL_STATE_HPP_
#define HUMANOID_PRO_RL_CONTROL_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "amp_policy_runner.h"

namespace humanoid_pro {
class RLControlState : public CopiedPolicyControlState {
private:
    static std::shared_ptr<PolicyRunnerBase> MakePolicy(RobotName robot_name) {
        auto policy = std::make_shared<AmpPolicyRunner>("amp", robot_name);
        policy->SetCmdMaxVel(Vec3f(3.0f, 1.0f, 1.2f));
        return policy;
    }
public:
    RLControlState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr, MakePolicy(robot_name),
                                   AmpControl_Walk2Run_Mode, {MimicControl_ChairDown_Mode, RLControl_Stand_Mode, AmpControl_Walk_Mode,
                                                   RLControl_StandSalute_Mode, RLControl_StandSaluteLeftArm_Mode,
                                                   RLControl_StandPlease_Mode, RLControl_StandTalking_Mode, RLControl_StandPlease2_Mode}) {}
};
}  // namespace humanoid_pro

#endif
