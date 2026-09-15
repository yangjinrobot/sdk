#ifndef HUMANOID_STAND_SHOOTING_STATE_HPP_
#define HUMANOID_STAND_SHOOTING_STATE_HPP_

#include "state_base.h"
#include "stand_shooting_policy_runner.h"

namespace humanoid {
class StandShootingState : public StateBase {
public:
    StandShootingState(const RobotName& robot_name, const std::string& state_name,
                       std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr),
          policy_("stand_shooting", robot_name,
                  "/opt/robot/share/motion_master/policy/915.onnx") {}

    void OnEnter() override {
        policy_.OnEnter();
        entry_time_ = ri_ptr_->GetInterfaceTimeStamp();
        entry_joint_pos_ = ri_ptr_->GetJointPosition();
        arm_blend_ = 0.f;
        arm_start_blend_ = 0.f;
        arm_target_blend_ = 0.f;
        arm_request_raised_ = false;
        arm_transition_time_ = entry_time_;
        run_count_ = 0;
        joint_cmd_.setZero(cp_ptr_->dof_num_, 5);
        msfb_.UpdateCurrentState(RobotMotionState::StandShooting);
    }
    void OnExit() override { policy_.OnExit(); }

    void Run() override {
        RobotBasicState state;
        state.base_rpy = ri_ptr_->GetImuRpy();
        state.base_rot_mat = RpyToRm(state.base_rpy);
        state.base_omega = ri_ptr_->GetImuOmega();
        state.joint_pos = ri_ptr_->GetJointPosition();
        state.joint_vel = ri_ptr_->GetJointVelocity();
        const float now = ri_ptr_->GetInterfaceTimeStamp();
        if (run_count_ % policy_.decimation_ == 0) {
            RobotAction action = policy_.GetRobotAction(state, *uc_ptr_->GetUserCommand());
            joint_cmd_ = action.ConvertToMat();
        }

        const bool request_raised = uc_ptr_->GetUserCommand()->target_action == 1;
        if (request_raised != arm_request_raised_) {
            UpdateArmBlend(now);
            arm_start_blend_ = arm_blend_;
            arm_target_blend_ = request_raised ? 1.f : 0.f;
            arm_transition_time_ = now;
            arm_request_raised_ = request_raised;
        }
        UpdateArmBlend(now);
        // The simulation's shooting pose changes the right elbow only:
        // key 2: 1.4 rad down; key 1: -0.2 rad raised.
        joint_cmd_(8, 1) = 1.4f + arm_blend_ * (-0.2f - 1.4f);

        // Entering from ZeroPos must not create a position step. Blend all
        // policy targets from the measured entry posture over three seconds.
        const float entry_blend = SmoothStep5((now - entry_time_) / 3.f);
        joint_cmd_.col(1) = entry_joint_pos_ + entry_blend * (joint_cmd_.col(1) - entry_joint_pos_);
        ri_ptr_->SetJointCommand(joint_cmd_);
        ++run_count_;
    }

    bool LoseControlJudge() override {
        return uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::JointDamping;
    }
    StateName GetNextStateName() override {
        const uint32_t target = uc_ptr_->GetUserCommand()->target_mode;
        if (target == RobotMotionState::JointDamping || uc_ptr_->GetUserCommand()->safe_control_mode != 0)
            return StateName::kJointDamping;
        if (target == RobotMotionState::RLControlMode) return StateName::kRLControl;
        if (target == RobotMotionState::Ysc) return StateName::kYsc;
        return StateName::kStandShooting;
    }

private:
    static float SmoothStep5(float x) {
        x = std::max(0.f, std::min(1.f, x));
        return 10.f * x * x * x - 15.f * x * x * x * x + 6.f * x * x * x * x * x;
    }
    void UpdateArmBlend(float now) {
        const float progress = SmoothStep5((now - arm_transition_time_) / 3.f);
        arm_blend_ = arm_start_blend_ + progress * (arm_target_blend_ - arm_start_blend_);
    }

    StandShootingPolicyRunner policy_;
    MatXf joint_cmd_;
    VecXf entry_joint_pos_;
    float entry_time_ = 0.f, arm_blend_ = 0.f, arm_start_blend_ = 0.f;
    float arm_target_blend_ = 0.f, arm_transition_time_ = 0.f;
    bool arm_request_raised_ = false;
    int run_count_ = 0;
};
}  // namespace humanoid

#endif
