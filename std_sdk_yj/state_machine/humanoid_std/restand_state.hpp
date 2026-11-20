#pragma once

#include "state_base.h"
#include "rl_stand_policy_runner.hpp"

namespace humanoid_std {
class RestandState final : public StateBase {
    RLStandPolicyRunner policy_;
    MatXf command_;
    MatXf entry_arms_;
    int ticks_ = 0;
    bool failed_ = false;
    double entry_time_ = 0;
    RLStandPolicyRunner::ArmPosture arm_posture_ = RLStandPolicyRunner::ArmPosture::Raised;
    Vec2f elbow_entry_ = Vec2f::Zero();
    double elbow_entry_time_ = 0;
    bool elbow_transition_active_ = false;
    static constexpr double kArmTransitionSeconds = 1.0;

    void Fail(const std::exception& error) {
        failed_ = true;
        std::cerr << "[restand] " << error.what() << "; entering damping\n";
    }

public:
    RestandState(const RobotName& robot, const std::string& name,
                 std::shared_ptr<ControllerData> data,
                 const std::filesystem::path& path = RLStandPolicyRunner::DefaultPolicyPath())
        : StateBase(robot, name, data), policy_(robot, path) {
        if (cp_ptr_->dof_num_ != 21) throw std::invalid_argument("restand requires 21 DOF");
        policy_.DisplayPolicyInfo();
    }

    void OnEnter() override {
        arm_posture_ = RLStandPolicyRunner::ArmPosture::Raised;
        policy_.SetArmPosture(arm_posture_);
        elbow_transition_active_ = false;
        policy_.OnEnter();
        ticks_ = 0;
        failed_ = false;
        auto& user = *uc_ptr_->GetUserCommand();
        user.forward_vel_scale = user.side_vel_scale = user.turnning_vel_scale = 0;
        msfb_.UpdateCurrentState(RobotMotionState::Restand);
        msfb_.UpdateCurrentGait(0);
        try {
            const VecXf position = ri_ptr_->GetJointPosition();
            if (position.size() != 21 || !position.allFinite())
                throw std::runtime_error("invalid restand entry position");
            const MatXf previous = ri_ptr_->GetJointCommand();
            if (previous.rows() != 21 || previous.cols() != 5 || !previous.allFinite())
                throw std::runtime_error("invalid restand previous joint command");
            entry_arms_ = previous.block(1, 0, 8, 5);
            entry_time_ = ri_ptr_->GetInterfaceTimeStamp();
        } catch (const std::exception& error) { Fail(error); }
    }

    void OnExit() override { policy_.OnExit(); }

    void Run() override {
        if (LoseControlJudge() || uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::ZeroPos) return;
        try {
            const uint32_t request = uc_ptr_->GetUserCommand()->target_mode;
            if (IsStandElbowCommand(request)) {
                const auto posture = request == RobotMotionCommand::StandElbowPlus12
                    ? RLStandPolicyRunner::ArmPosture::Lowered : RLStandPolicyRunner::ArmPosture::Raised;
                if (posture != arm_posture_) {
                    const MatXf previous = ri_ptr_->GetJointCommand();
                    if (previous.rows() != 21 || previous.cols() != 5 || !previous.allFinite())
                        throw std::runtime_error("invalid previous elbow command");
                    elbow_entry_ << previous(4, 1), previous(8, 1);
                    elbow_entry_time_ = ri_ptr_->GetInterfaceTimeStamp();
                    elbow_transition_active_ = true;
                    arm_posture_ = posture;
                    policy_.SetArmPosture(posture);
                }
            }
            if (ticks_ == 0) {
                RobotBasicState state;
                state.joint_pos = ri_ptr_->GetJointPosition();
                state.joint_vel = ri_ptr_->GetJointVelocity();
                state.base_rpy = ri_ptr_->GetImuRpy();
                state.base_rot_mat = RpyToRm(state.base_rpy);
                state.base_omega = ri_ptr_->GetImuOmega();
                // Restand 始终原地站立，手柄后续更新不能给策略注入速度。
                UserCommand stand_command = *uc_ptr_->GetUserCommand();
                stand_command.forward_vel_scale = 0;
                stand_command.side_vel_scale = 0;
                stand_command.turnning_vel_scale = 0;
                command_ = policy_.GetRobotAction(state, stand_command).ConvertToMat();
            }
            MatXf output = command_;
            const float s = static_cast<float>(std::clamp(
                (ri_ptr_->GetInterfaceTimeStamp() - entry_time_) / kArmTransitionSeconds, 0.0, 1.0));
            const float weight = s * s * (3.0f - 2.0f * s);
            // Blend kp/position/kd/velocity/torque from the last published command.
            output.block(1, 0, 8, 5) = (1 - weight) * entry_arms_ + weight * command_.block(1, 0, 8, 5);
            if (elbow_transition_active_) {
                const float t = static_cast<float>(std::clamp(
                    (ri_ptr_->GetInterfaceTimeStamp() - elbow_entry_time_) / kArmTransitionSeconds, 0.0, 1.0));
                const float w = t * t * (3.0f - 2.0f * t);
                const float target = arm_posture_ == RLStandPolicyRunner::ArmPosture::Lowered ? 1.2f : -.4f;
                output(4, 1) = (1 - w) * elbow_entry_(0) + w * target;
                output(8, 1) = (1 - w) * elbow_entry_(1) + w * target;
                if (t >= 1.0f) elbow_transition_active_ = false;
            }
            ri_ptr_->SetJointCommand(output);
            ticks_ = (ticks_ + 1) % policy_.decimation_;
        } catch (const std::exception& error) { Fail(error); }
    }

    bool LoseControlJudge() override {
        const auto& user = *uc_ptr_->GetUserCommand();
        return failed_ || user.safe_control_mode != 0 || user.target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        if (LoseControlJudge()) return StateName::kJointDamping;
        switch (uc_ptr_->GetUserCommand()->target_mode) {
            case RobotMotionState::ZeroPos: return StateName::kZeroPos;
            case RobotMotionState::RLControlMode: return StateName::kRLControl;
            case RobotMotionState::Run: return StateName::kRun;
            case RobotMotionState::Stand2SitMimic: return StateName::kStand2SitMimic;
            case RobotMotionState::SuspendedStand: return StateName::kSuspendedStand;
            case RobotMotionState::WaitingForStand: return StateName::kIdle;
            default: return StateName::kRestand;
        }
    }
};
} // namespace humanoid_std
