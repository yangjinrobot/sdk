/** Dedicated RL state for restand.onnx. */
#ifndef HUMANOID_RESTAND_STATE_HPP_
#define HUMANOID_RESTAND_STATE_HPP_

#include "humanoid/rl_control_state.hpp"
#include "restand_policy_runner.h"

namespace humanoid {
class ReStandState : public RLControlState {
public:
    ReStandState(const RobotName& robot_name, const std::string& state_name,
                 std::shared_ptr<ControllerData> data_ptr)
        : RLControlState(
              robot_name, state_name, data_ptr,
              "/opt/robot/share/motion_master/policy/restand.onnx",
              RobotMotionState::ReStand, StateName::kReStand,
              std::make_shared<ReStandPolicyRunner>(
                  "restand", robot_name,
                  "/opt/robot/share/motion_master/policy/restand.onnx")) {}
};
}  // namespace humanoid

#endif
