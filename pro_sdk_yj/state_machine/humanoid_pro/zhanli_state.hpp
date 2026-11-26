#ifndef HUMANOID_PRO_ZHANLI_STATE_HPP_
#define HUMANOID_PRO_ZHANLI_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "zhanli_policy_runner.h"

namespace humanoid_pro {
// Zhanli is reachable only from Tangxia and returns directly to AMP walking.
class ZhanliState : public CopiedPolicyControlState {
private:
    static std::shared_ptr<PolicyRunnerBase> MakePolicy(RobotName robot_name) {
        return std::make_shared<ZhanliPolicyRunner>("zhanli", robot_name);
    }

public:
    ZhanliState(RobotName robot_name, const std::string& state_name,
                std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr, MakePolicy(robot_name),
                                   MimicControl_GetUp_Mode, {AmpControl_Walk_Mode}) {}
};
}  // namespace humanoid_pro

#endif
