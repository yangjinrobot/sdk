/** YSC policy state driven by policy_CR1_8.onnx. */
#ifndef HUMANOID_YSC_STATE_HPP_
#define HUMANOID_YSC_STATE_HPP_

#include "state_base.h"
#include "ysc_policy_runner.h"

namespace humanoid {
class YscState : public StateBase {
public:
    YscState(const RobotName& robot_name, const std::string& state_name,
             std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr),
          policy_("ysc", robot_name,
                  "/opt/robot/share/motion_master/policy/policy_CR1_8.onnx") {}

    void OnEnter() override {
        policy_.OnEnter();
        run_count_ = 0;
        joint_cmd_.setZero(cp_ptr_->dof_num_, 5);
        msfb_.UpdateCurrentState(RobotMotionState::Ysc);
    }

    void OnExit() override { policy_.OnExit(); }

    void Run() override {
        if (run_count_ % policy_.decimation_ == 0) {
            RobotBasicState state;
            state.base_rpy = ri_ptr_->GetImuRpy();
            state.base_rot_mat = RpyToRm(state.base_rpy);
            state.base_omega = ri_ptr_->GetImuOmega();
            state.joint_pos = ri_ptr_->GetJointPosition();
            state.joint_vel = ri_ptr_->GetJointVelocity();
            joint_cmd_ = policy_.GetRobotAction(state, *uc_ptr_->GetUserCommand()).ConvertToMat();
        }
        ri_ptr_->SetJointCommand(joint_cmd_);
        ++run_count_;
    }

    bool LoseControlJudge() override {
        return uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        const uint32_t target = uc_ptr_->GetUserCommand()->target_mode;
        const uint32_t safe_mode = uc_ptr_->GetUserCommand()->safe_control_mode;
        const uint32_t non_posture_faults = safe_mode & ~(1u << kSafePostureNormal);
        if (target == RobotMotionState::JointDamping || non_posture_faults != 0)
            return StateName::kJointDamping;
        if (target == RobotMotionState::RLControlMode) return StateName::kRLControl;
        if (target == RobotMotionState::WaitingForStand) return StateName::kIdle;
        return StateName::kYsc;
    }

private:
    YscPolicyRunner policy_;
    MatXf joint_cmd_;
    int run_count_ = 0;
};
}  // namespace humanoid

#endif
