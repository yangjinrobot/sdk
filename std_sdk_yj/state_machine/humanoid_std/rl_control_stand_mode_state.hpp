#pragma once

#include "state_base.h"
#include "rl_control_stand_mode_policy_runner.hpp"

namespace humanoid_std {
class RLControlStandModeState final : public StateBase {
    RLControlStandModePolicyRunner policy_;
    MatXf command_;
    MatXf entry_arms_;
    int ticks_ = 0;
    bool failed_ = false;
    double entry_time_ = 0;
    RLControlStandModePolicyRunner::ArmPosture arm_posture_ = RLControlStandModePolicyRunner::ArmPosture::Lowered;
    Vec2f elbow_entry_ = Vec2f::Zero();
    double elbow_entry_time_ = 0;
    bool elbow_transition_active_ = false;
    RobotMotionState auto_exit_target_ = RobotMotionState::WaitingForStand;
    RobotMotionState requested_target_ = RobotMotionState::WaitingForStand;
    static constexpr double kArmTransitionSeconds = 1.0;
    static constexpr double kAutoExitSettleSeconds = 0.02;

    bool EntryBlendComplete() const {
        return ri_ptr_->GetInterfaceTimeStamp() - entry_time_ >=
               kArmTransitionSeconds + kAutoExitSettleSeconds;
    }

    void Fail(const std::exception& error) {
        failed_ = true;
        std::cerr << "[rl_control_stand_mode] " << error.what() << "; entering damping\n";
    }

public:
    RLControlStandModeState(const RobotName& robot, const std::string& name,
                 std::shared_ptr<ControllerData> data,
                 const std::filesystem::path& path = RLControlStandModePolicyRunner::DefaultPolicyPath())
        : StateBase(robot, name, data), policy_(robot, path) {
        if (cp_ptr_->dof_num_ != 21) throw std::invalid_argument("rl_control_stand_mode requires 21 DOF");
        policy_.DisplayPolicyInfo();
    }

    void OnEnter() override {
        const uint32_t source = msfb_.GetCurrentState();
        auto& user = *uc_ptr_->GetUserCommand();
        requested_target_ = static_cast<RobotMotionState>(user.target_mode);
        const bool chair_from_rl =
            source == RobotMotionState::AmpControl_NiRenWalk_Mode &&
            requested_target_ == RobotMotionState::MimicControl_ChairDown_Mode;
        auto_exit_target_ = chair_from_rl
            ? RobotMotionState::AmpControl_GunShootWalk_Mode
            : RequiresRLControlStandTransition(source, user.target_mode)
                ? requested_target_ : RobotMotionState::WaitingForStand;
        // Automatic transitions prepare the elbows inside Stand itself. GunShoot
        // uses the -0.4 hand posture; the walking modes use +1.2. Manual Stand
        // entry retains the source-based posture selection.
        const bool raised = auto_exit_target_ != RobotMotionState::WaitingForStand
            ? auto_exit_target_ == RobotMotionState::AmpControl_GunShootWalk_Mode
            : source == RobotMotionState::AmpControl_Walk2Run_Mode ||
                source == RobotMotionState::MimicControl_ChairUp_Mode;
        arm_posture_ = raised ? RLControlStandModePolicyRunner::ArmPosture::Raised
                              : RLControlStandModePolicyRunner::ArmPosture::Lowered;
        policy_.SetArmPosture(arm_posture_);
        elbow_transition_active_ = false;
        policy_.OnEnter();
        ticks_ = 0;
        failed_ = false;
        user.forward_vel_scale = user.side_vel_scale = user.turnning_vel_scale = 0;
        msfb_.UpdateCurrentState(RobotMotionState::RLControl_Stand_Mode);
        msfb_.UpdateCurrentGait(0);
        try {
            const VecXf position = ri_ptr_->GetJointPosition();
            if (position.size() != 21 || !position.allFinite())
                throw std::runtime_error("invalid rl_control_stand_mode entry position");
            const MatXf previous = ri_ptr_->GetJointCommand();
            if (previous.rows() != 21 || previous.cols() != 5 || !previous.allFinite())
                throw std::runtime_error("invalid rl_control_stand_mode previous joint command");
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
                    ? RLControlStandModePolicyRunner::ArmPosture::Lowered : RLControlStandModePolicyRunner::ArmPosture::Raised;
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
                // RLControl_Stand_Mode 始终原地站立，手柄后续更新不能给策略注入速度。
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
                const float target = arm_posture_ == RLControlStandModePolicyRunner::ArmPosture::Lowered ? 1.2f : -.4f;
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
        const uint32_t target = uc_ptr_->GetUserCommand()->target_mode;
        if (auto_exit_target_ != RobotMotionState::WaitingForStand) {
            if (target != requested_target_) {
                auto_exit_target_ = RobotMotionState::WaitingForStand;
            } else {
                if (!EntryBlendComplete()) return StateName::kRLControlStandMode;
                switch (auto_exit_target_) {
                    case RobotMotionState::AmpControl_NiRenWalk_Mode:
                        return StateName::kAmpControlNiRenWalkMode;
                    case RobotMotionState::AmpControl_GunShootWalk_Mode:
                        return StateName::kAmpControlGunShootWalkMode;
                    case RobotMotionState::AmpControl_Walk2Run_Mode:
                        return StateName::kAmpControlWalk2RunMode;
                    default:
                        break;
                }
            }
        }
        switch (target) {
            case RobotMotionState::ZeroPos: return StateName::kZeroPos;
            case RobotMotionState::AmpControl_GunShootWalk_Mode: return StateName::kAmpControlGunShootWalkMode;
            case RobotMotionState::AmpControl_NiRenWalk_Mode: return StateName::kAmpControlNiRenWalkMode;
            case RobotMotionState::AmpControl_Walk2Run_Mode: return StateName::kAmpControlWalk2RunMode;
            case RobotMotionState::MjlabControl_Walk2Run_Mode: return StateName::kMjlabControlWalk2RunMode;
            case RobotMotionState::MimicControl_Kneel_Mode: return StateName::kMimicControlKneelMode;
            case RobotMotionState::MimicControl_ChairDown_Mode: return StateName::kMimicControlChairDownMode;
            case RobotMotionState::SuspendedStand: return StateName::kSuspendedStand;
            case RobotMotionState::WaitingForStand: return StateName::kIdle;
            default: return StateName::kRLControlStandMode;
        }
    }
};
} // namespace humanoid_std
