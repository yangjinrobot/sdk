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
                                   RLControlMode, {RLControlMimic12Mode, RLStandControlMode, AmpWalkingControl,
                                                   RLStandSaluteMode, RLStandSaluteLeftArmMode,
                                                   RLStandPleaseMode, RLStandTalkingMode, RLStandPlease2Mode}) {}
};
}  // namespace humanoid_pro

#endif
