#ifndef HUMANOID_PRO_RL_STAND_SALUTE_STATE_HPP_
#define HUMANOID_PRO_RL_STAND_SALUTE_STATE_HPP_

#include "humanoid_pro/rl_stand_control_state.hpp"
#include "rl_stand_salute_policy_runner.hpp"

namespace humanoid_pro {
class RLStandSaluteState : public RLStandControlState {
private:
    std::shared_ptr<RLStandSalutePolicyRunner> salute_policy_;
    bool walking_requested_ = false;
    bool gesture_transition_ = false;
    const RobotMotionState salute_mode_;

public:
    RLStandSaluteState(const RobotName& robot_name, const std::string& state_name,
                      std::shared_ptr<ControllerData> data_ptr, std::shared_ptr<RLStandSalutePolicyRunner> policy,
                      RobotMotionState mode = RLStandSaluteMode)
        : RLStandControlState(robot_name,state_name,data_ptr,policy,mode), salute_policy_(std::move(policy)), salute_mode_(mode) {}

    RLStandSaluteState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : RLStandSaluteState(robot_name,state_name,data_ptr,
              std::make_shared<RLStandSalutePolicyRunner>("rl_stand_salute",robot_name)) {}

    void OnEnter() override {
        walking_requested_ = false;
        gesture_transition_ = false;
        salute_policy_->SetLeftArmMotionEnabled(salute_mode_ == RLStandSaluteLeftArmMode);
        RLStandControlState::OnEnter();
    }
    void OnExit() override {
        salute_policy_->PreservePolicyOnExit(gesture_transition_);
        RLStandControlState::OnExit();
        walking_requested_ = false;
        gesture_transition_ = false;
    }
    StateName GetNextStateName() override {
        gesture_transition_ = false;
        const auto next = RLStandControlState::GetNextStateName();
        // Safety always preempts the pending walking transition.
        if (next == kJointDamping) return next;
        // The operator can take over with RL Control at any gesture phase,
        // including while arm lowering is delaying entry into AMP Walking.
        if (next == kRLControl) return next;
        if (next == kAmpWalkingControl) {
            walking_requested_ = true;
            salute_policy_->RequestArmLowering();
        }
        // Keep the current state and its standing policy active until the
        // right arm trajectory and measured posture have both completed.
        if (walking_requested_)
            return salute_policy_->IsArmLoweringComplete() ? kAmpWalkingControl : static_cast<StateName>(salute_mode_);
        if ((next == kRLStandSaluteLeftArm && salute_mode_ == RLStandSaluteMode) ||
            (next == kRLStandSalute && salute_mode_ == RLStandSaluteLeftArmMode)) {
            gesture_transition_ = true;
        }
        return next;
    }
};
}  // namespace humanoid_pro

#endif
