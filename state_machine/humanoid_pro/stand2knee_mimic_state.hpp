#ifndef HUMANOID_PRO_STAND2KNEE_MIMIC_STATE_HPP_
#define HUMANOID_PRO_STAND2KNEE_MIMIC_STATE_HPP_

#include "humanoid_pro/copied_policy_control_state.hpp"
#include "knee_mimic_policy_runner.h"

namespace humanoid_pro {
class Stand2KneeMimicState : public CopiedPolicyControlState {
public:
    Stand2KneeMimicState(RobotName robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : CopiedPolicyControlState(robot_name, state_name, data_ptr,
              // At frame 383 the runner masks arm observations/actions and
              // hands both arms to the planner: 1=forward, 2=spread, shared UDP.
              std::make_shared<Stand2KneePolicyRunner>("stand2knee", robot_name, "", data_ptr->udp_node),
              Stand2KneeMimicMode, {Knee2StandMimicMode}) {}
};
}  // namespace humanoid_pro

#endif
