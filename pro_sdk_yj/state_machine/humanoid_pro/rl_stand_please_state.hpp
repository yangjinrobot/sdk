#ifndef HUMANOID_PRO_RL_STAND_PLEASE_STATE_HPP_
#define HUMANOID_PRO_RL_STAND_PLEASE_STATE_HPP_

#include "humanoid_pro/rl_stand_control_state.hpp"
#include "rl_stand_please_policy_runner.hpp"

namespace humanoid_pro {
class RLStandPleaseState : public RLStandControlState {
private:
    std::shared_ptr<RLStandPleasePolicyRunner> please_policy_;
    bool walking_requested_ = false;
    const RobotMotionState please_mode_;

public:
    RLStandPleaseState(const RobotName& robot_name, const std::string& state_name,
                      std::shared_ptr<ControllerData> data_ptr, std::shared_ptr<RLStandPleasePolicyRunner> policy,
                      RobotMotionState mode = RLStandPleaseMode)
        : RLStandControlState(robot_name,state_name,data_ptr,policy,mode), please_policy_(std::move(policy)), please_mode_(mode) {}

    RLStandPleaseState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : RLStandPleaseState(robot_name,state_name,data_ptr,
              std::make_shared<RLStandPleasePolicyRunner>("rl_stand_please",robot_name)) {}

    void OnEnter() override {
        walking_requested_ = false;
        // The please gesture drives both arms, so there is no arm-mode switch.
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
            please_policy_->RequestArmLowering();
        }
        // Keep the current state and its standing policy active until the
        // right arm trajectory and measured posture have both completed.
        if (walking_requested_)
            return please_policy_->IsArmLoweringComplete() ? kAmpWalkingControl : static_cast<StateName>(please_mode_);
        return next;
    }
};
}  // namespace humanoid_pro

#endif
