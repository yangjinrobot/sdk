#ifndef HUMANOID_PRO_TANGXIA_STATE_HPP_
#define HUMANOID_PRO_TANGXIA_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "tangxia_policy_runner.h"

namespace humanoid_pro {
// Tangxia is reachable only from AmpWalking. It holds the last lie-down
// reference until a Zhanli request is accepted.
class TangxiaState : public CopiedPolicyControlState {
private:
    static std::shared_ptr<PolicyRunnerBase> MakePolicy(RobotName robot_name) {
        return std::make_shared<TangxiaPolicyRunner>("tangxia", robot_name);
    }

public:
    TangxiaState(RobotName robot_name, const std::string& state_name,
                 std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr, MakePolicy(robot_name),
                                   MimicControl_LieDown_Mode, {MimicControl_GetUp_Mode}, RobotMotionState::AmpControl_Walk_Mode) {}
};
}  // namespace humanoid_pro

#endif
