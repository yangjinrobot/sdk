#pragma once

#include "state_base.h"
#include "sit2stand_mimic_policy_runner.hpp"
#include <chrono>

namespace humanoid_std {

// Enter from completed stand2sit using the measured seated pose. Play once,
// allow requested transitions after 200 frames; otherwise play to final hold.
// No simulated pose reset or interpolation to a standing default on entry.
class Sit2StandMimicState : public StateBase {
    std::shared_ptr<Sit2StandMimicPolicyRunner> policy_;
    RobotBasicState robot_{};
    MatXf command_;
    int ticks_ = 0;
    bool failed_ = false;
    bool from_squat_preparation_ = false;
    static constexpr int kSwitchAfterFrames = 200;

    void ReadObservation() {
        robot_.joint_pos = ri_ptr_->GetJointPosition();
        robot_.joint_vel = ri_ptr_->GetJointVelocity();
        robot_.base_rpy = ri_ptr_->GetImuRpy();
        robot_.base_rot_mat = RpyToRm(robot_.base_rpy);
        robot_.base_omega = ri_ptr_->GetImuOmega();
        if (robot_.joint_pos.size() != 21 || robot_.joint_vel.size() != 21 ||
            !robot_.joint_pos.allFinite() || !robot_.joint_vel.allFinite() ||
            !robot_.base_rot_mat.allFinite() || !robot_.base_omega.allFinite())
            throw std::runtime_error("Invalid sit2stand feedback");
    }

    void Fail(const std::exception& error) {
        failed_ = true;
        std::cerr << "[SIT2STAND_MIMIC] " << error.what() << "; entering damping\n";
    }

public:
    Sit2StandMimicState(const RobotName& robot, const std::string& name,
                       std::shared_ptr<ControllerData> data)
        : StateBase(robot, name, data) {
        if (robot != RobotName::CR1STANDARD || cp_ptr_->dof_num_ != 21)
            throw std::invalid_argument("Sit2stand mimic requires CR1STANDARD / 21 DOF");
        policy_ = std::make_shared<Sit2StandMimicPolicyRunner>(robot);
        policy_->DisplayPolicyInfo();
    }

    void OnEnter() override {
        from_squat_preparation_ = msfb_.GetCurrentState() == RobotMotionState::SquatPreparation;
        msfb_.UpdateCurrentState(RobotMotionState::Sit2StandMimic);
        msfb_.SetHandsetReady(false);
        ticks_ = 0;
        failed_ = false;
        auto& user = *uc_ptr_->GetUserCommand();
        user.forward_vel_scale = user.side_vel_scale = user.turnning_vel_scale = 0.0f;
        msfb_.UpdateCurrentGait(0);
        policy_->OnEnter();
        try {
            ReadObservation();
            policy_->AlignHeading(robot_.base_rot_mat);
            std::cout << "[SIT2STAND_MIMIC] Playing " << policy_->FrameCount()
                      << " embedded ONNX frames from frame 0; transitions allowed after "
                      << (from_squat_preparation_ ? policy_->FrameCount() : kSwitchAfterFrames)
                      << " frames; hold final frame if no request; T sits / C RL walks / V run / restand, Z zeropos, R damping\n";
        } catch (const std::exception& error) { Fail(error); }
    }

    void OnExit() override {
        policy_->OnExit();
        msfb_.SetHandsetReady(false);
        msfb_.UpdateCurrentGait(0);
    }

    void Run() override {
        if (LoseControlJudge() || uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::ZeroPos) return;
        try {
            ReadObservation();
            if (ticks_ == 0) {
                const auto start = std::chrono::steady_clock::now();
                command_ = policy_->GetRobotAction(robot_, *uc_ptr_->GetUserCommand()).ConvertToMat();
#ifndef MOTION_MASTER_MUJOCO_LOCKSTEP
                if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(20))
                    throw std::runtime_error("Sit2stand inference exceeded 20 ms");
#endif
            }
            ticks_ = (ticks_ + 1) % policy_->decimation_;
            ri_ptr_->SetJointCommand(command_);
            if (policy_->IsMotionFinished() && ticks_ == 0) msfb_.SetHandsetReady(true);
        } catch (const std::exception& error) { Fail(error); }
    }

    bool LoseControlJudge() override {
        const auto& user = *uc_ptr_->GetUserCommand();
        return failed_ || user.safe_control_mode != 0 || user.target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        if (LoseControlJudge()) return StateName::kJointDamping;
        if (uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::ZeroPos) return StateName::kZeroPos;
        // Frame() advances after inference: 200 means frames 0..199 were
        // inferred. Wait for ticks_ == 0 so frame 199 executes a full 20 ms.
        const bool can_switch = policy_->IsMotionFinished() ||
            (!from_squat_preparation_ && policy_->Frame() >= kSwitchAfterFrames);
        if (can_switch && ticks_ == 0) {
            const auto target = uc_ptr_->GetUserCommand()->target_mode;
            if (target == RobotMotionState::Stand2SitMimic) return StateName::kStand2SitMimic;
            if (target == RobotMotionState::RLControlMode) return StateName::kRLControl;
            if (target == RobotMotionState::Run) return StateName::kRun;
            if (target == RobotMotionState::Restand) return StateName::kRestand;
        }
        return StateName::kSit2StandMimic;
    }
};
} // namespace humanoid_std
