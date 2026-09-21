#ifndef HUMANOID_ZHANLI_SHOOTING_STATE_HPP_
#define HUMANOID_ZHANLI_SHOOTING_STATE_HPP_

#include "state_base.h"
#include "zhanli_shooting_policy_runner.hpp"

namespace humanoid {

class ZhanliShootingState : public StateBase {
public:
    ZhanliShootingState(const RobotName& robot_name, const std::string& state_name,
                         std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr),
          policy_("zhanli_shooting", robot_name,
                  "/opt/robot/share/motion_master/policy/sheji_0920.onnx") {
        // 站立射击策略复用了步态策略的 UserCommand 速度输入，
        // 必须显式设置各方向最大速度，否则 cmd_vel 会因 max_cmd_vel_ 默认
        // 为 0 而被全部置零，导致手柄/App 速度指令无效。
        policy_.SetCmdMaxVel(Vec3f(1.0f, 0.5f, 1.0f));
    }

    void OnEnter() override {
        policy_.OnEnter();
        run_count_ = 0;
        joint_cmd_.setZero(cp_ptr_->dof_num_, 5);
        msfb_.UpdateCurrentState(RobotMotionState::ZhanliShooting);
    }

    void OnExit() override { policy_.OnExit(); }

    void Run() override {
        RobotBasicState state;
        state.base_rpy = ri_ptr_->GetImuRpy();
        state.base_rot_mat = RpyToRm(state.base_rpy);
        state.base_omega = ri_ptr_->GetImuOmega();
        state.joint_pos = ri_ptr_->GetJointPosition();
        state.joint_vel = ri_ptr_->GetJointVelocity();
        if (run_count_ % policy_.decimation_ == 0)
            joint_cmd_ = policy_.GetRobotAction(state, *uc_ptr_->GetUserCommand()).ConvertToMat();

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
        if (target == RobotMotionState::ReStand) return StateName::kReStand;
        return StateName::kZhanliShooting;
    }

private:
    ZhanliShootingPolicyRunner policy_;
    MatXf joint_cmd_;
    int run_count_ = 0;
};

}  // namespace humanoid

#endif
