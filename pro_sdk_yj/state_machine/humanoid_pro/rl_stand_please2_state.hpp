#ifndef HUMANOID_PRO_RL_STAND_PLEASE2_STATE_HPP_
#define HUMANOID_PRO_RL_STAND_PLEASE2_STATE_HPP_

#include "humanoid_pro/rl_stand_control_state.hpp"
#include "rl_stand_please2_policy_runner.hpp"

namespace humanoid_pro {
class RLStandPlease2State : public RLStandControlState {
private:
    std::shared_ptr<RLStandPlease2PolicyRunner> please2_policy_;
    bool walking_requested_ = false;
    const RobotMotionState please2_mode_;

public:
    RLStandPlease2State(const RobotName& robot_name, const std::string& state_name,
                       std::shared_ptr<ControllerData> data_ptr, std::shared_ptr<RLStandPlease2PolicyRunner> policy,
                       RobotMotionState mode = RLControl_StandPlease2_Mode)
        : RLStandControlState(robot_name,state_name,data_ptr,policy,mode), please2_policy_(std::move(policy)), please2_mode_(mode) {}

    RLStandPlease2State(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : RLStandPlease2State(robot_name,state_name,data_ptr,
              std::make_shared<RLStandPlease2PolicyRunner>("rl_stand_please2",robot_name)) {}

    void OnEnter() override {
        walking_requested_ = false;
        // The please2 gesture drives both arms, so there is no arm-mode switch.
        RLStandControlState::OnEnter();
    }
    void OnExit() override {
        RLStandControlState::OnExit();
        walking_requested_ = false;
    }
    StateName GetNextStateName() override {
        const auto next = RLStandControlState::GetNextStateName();
        // Safety always preempts the pending walking transition.
        if (next == kJointDamping) return next;
        // The operator can take over with RL Control at any gesture phase,
        // including while arm lowering is delaying entry into AMP Walking.
        if (next == kRLControl) return next;
        if (next == kAmpWalkingControl) {
            walking_requested_ = true;
            please2_policy_->RequestArmLowering();
        }
        // Keep the current state and its standing policy active until the
        // arm trajectories and measured posture have both completed.
        if (walking_requested_)
            return please2_policy_->IsArmLoweringComplete() ? kAmpWalkingControl : static_cast<StateName>(please2_mode_);
        return next;
    }
};
}  // namespace humanoid_pro

#endif
