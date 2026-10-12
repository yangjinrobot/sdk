#ifndef HUMANOID_NIRENWALK_STATE_HPP_
#define HUMANOID_NIRENWALK_STATE_HPP_

#include "state_base.h"
#include "nirenwalk_policy_runner.h"
#include <chrono>

namespace humanoid {
class NirenWalkState final : public StateBase {
public:
    NirenWalkState(const RobotName& robot_name, const std::string& state_name,
                   std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr),
          policy_("nirenwalk", robot_name,
                  "/opt/robot/share/motion_master/policy/nirenxingzou.onnx") {}

    void OnEnter() override {
        policy_.OnEnter();
        run_count_ = 0;
        failed_ = false;
        arm_entry_pos_ = ri_ptr_->GetJointPosition().segment(1, 8);
        arm_entry_transition_active_ = arm_entry_pos_.allFinite();
        arm_entry_transition_started_ = false;
        joint_cmd_ = ri_ptr_->GetJointCommand();
        if (joint_cmd_.rows() != cp_ptr_->dof_num_ || joint_cmd_.cols() != 5 ||
            !joint_cmd_.allFinite()) {
            joint_cmd_.setZero(cp_ptr_->dof_num_, 5);
        }
        msfb_.UpdateCurrentState(RobotMotionState::NirenWalk);
        msfb_.UpdateCurrentGait(0);
        std::cout << "[nirenwalk] policy started at 50 Hz" << std::endl;
    }

    void OnExit() override { policy_.OnExit(); }

    void Run() override {
        if (LoseControlJudge()) return;
        if (run_count_ % policy_.decimation_ == 0) {
            RobotBasicState state;
            state.base_rpy = ri_ptr_->GetImuRpy();
            state.base_rot_mat = RpyToRm(state.base_rpy);
            state.base_omega = ri_ptr_->GetImuOmega();
            state.joint_pos = ri_ptr_->GetJointPosition();
            state.joint_vel = ri_ptr_->GetJointVelocity();
            try {
                joint_cmd_ = policy_.GetRobotAction(state, *uc_ptr_->GetUserCommand()).ConvertToMat();
            } catch (const std::exception& error) {
                std::cerr << "[nirenwalk] " << error.what() << "; entering damping" << std::endl;
                failed_ = true;
                return;
            }
        }
        MatXf command = joint_cmd_;
        BlendArmEntryPosition(command);
        ri_ptr_->SetJointCommand(command);
        ++run_count_;
    }

    bool LoseControlJudge() override {
        const auto* command = uc_ptr_->GetUserCommand();
        return failed_ || command->safe_control_mode != 0 ||
               command->target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        if (LoseControlJudge()) return StateName::kJointDamping;
        switch (uc_ptr_->GetUserCommand()->target_mode) {
            case RobotMotionState::RLControlMode: return StateName::kRLControl;
            case RobotMotionState::ZeroPos: return StateName::kZeroPos;
            default: return StateName::kNirenWalk;
        }
    }

private:
    static constexpr double kArmEntryTransitionDuration = 2.0;

    void BlendArmEntryPosition(MatXf& command) {
        if (!arm_entry_transition_active_) return;
        if (arm_entry_pos_.size() != 8 || command.rows() < 9 || command.cols() < 2) {
            arm_entry_transition_active_ = false;
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (!arm_entry_transition_started_) {
            arm_entry_transition_start_ = now;
            arm_entry_transition_started_ = true;
        }
        const double elapsed = std::chrono::duration<double>(now - arm_entry_transition_start_).count();
        const float s = static_cast<float>(std::min(1.0, elapsed / kArmEntryTransitionDuration));
        const float weight = s * s * (3.0f - 2.0f * s);
        for (int i = 0; i < 8; ++i) {
            command(1 + i, 1) = (1.0f - weight) * arm_entry_pos_(i) + weight * command(1 + i, 1);
        }
        if (s >= 1.0f) arm_entry_transition_active_ = false;
    }

    NirenWalkPolicyRunner policy_;
    MatXf joint_cmd_;
    int run_count_ = 0;
    bool failed_ = false;
    VecXf arm_entry_pos_;
    bool arm_entry_transition_active_ = false;
    bool arm_entry_transition_started_ = false;
    std::chrono::steady_clock::time_point arm_entry_transition_start_;
};
}  // namespace humanoid

#endif
