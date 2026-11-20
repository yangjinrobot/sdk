#ifndef HUMANOID_PRO_RL_STAND_TALKING_STATE_HPP_
#define HUMANOID_PRO_RL_STAND_TALKING_STATE_HPP_

#include "humanoid_pro/rl_stand_control_state.hpp"
#include "rl_stand_talking_policy_runner.hpp"

namespace humanoid_pro {
class RLStandTalkingState : public RLStandControlState {
private:
    std::shared_ptr<RLStandTalkingPolicyRunner> talking_policy_;
    bool walking_requested_ = false;
    const RobotMotionState talking_mode_;

public:
    RLStandTalkingState(const RobotName& robot_name, const std::string& state_name,
                      std::shared_ptr<ControllerData> data_ptr, std::shared_ptr<RLStandTalkingPolicyRunner> policy,
                      RobotMotionState mode = RLStandTalkingMode)
        : RLStandControlState(robot_name,state_name,data_ptr,policy,mode), talking_policy_(std::move(policy)), talking_mode_(mode) {}

    RLStandTalkingState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : RLStandTalkingState(robot_name,state_name,data_ptr,
              std::make_shared<RLStandTalkingPolicyRunner>("rl_stand_talking",robot_name)) {}

    void OnEnter() override {
        walking_requested_ = false;
        // The talking gesture drives both arms, so there is no arm-mode switch.
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
            talking_policy_->RequestArmLowering();
        }
        // Keep the current state and its standing policy active until the
        // right arm trajectory and measured posture have both completed.
        if (walking_requested_)
            return talking_policy_->IsArmLoweringComplete() ? kAmpWalkingControl : static_cast<StateName>(talking_mode_);
        return next;
    }
};
}  // namespace humanoid_pro

#endif
