#pragma once

#include "state_base.h"
#include "mimic_control_chair_up_mode_policy_runner.hpp"
#include "arm_entry_transition.hpp"
#include <chrono>

namespace humanoid_std {

// Enter from completed mimic_control_chair_down_mode using the measured seated pose. Play once,
// allow requested transitions after 200 frames; otherwise play to final hold.
// No simulated pose reset or interpolation to a standing default on entry.
class MimicControlChairUpModeState : public StateBase {
    std::shared_ptr<MimicControlChairUpModePolicyRunner> policy_;
    RobotBasicState robot_{};
    MatXf command_;
    ArmEntryTransition arm_entry_transition_;
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
            throw std::runtime_error("Invalid mimic_control_chair_up_mode feedback");
    }

    void Fail(const std::exception& error) {
        failed_ = true;
        std::cerr << "[MIMIC_CONTROL_CHAIR_UP_MODE] " << error.what() << "; entering damping\n";
    }

    void PrepareEntryTransition() {
        arm_entry_transition_.Cancel();
        const MatXf previous = ri_ptr_->GetJointCommand();
        if (previous.rows() != cp_ptr_->dof_num_ || previous.cols() != 5 || !previous.allFinite()) {
            std::cerr << "[MIMIC_CONTROL_CHAIR_UP_MODE] unavailable previous command; skipping arm entry blend\n";
            return;
        }
        arm_entry_transition_.Reset(previous.block(1, 0, 8, 5), ri_ptr_->GetInterfaceTimeStamp());
    }

public:
    MimicControlChairUpModeState(const RobotName& robot, const std::string& name,
                       std::shared_ptr<ControllerData> data)
        : StateBase(robot, name, data) {
        if (robot != RobotName::CR1STANDARD || cp_ptr_->dof_num_ != 21)
            throw std::invalid_argument("Sit2stand mimic requires CR1STANDARD / 21 DOF");
        policy_ = std::make_shared<MimicControlChairUpModePolicyRunner>(robot);
        policy_->DisplayPolicyInfo();
    }

    void OnEnter() override {
        from_squat_preparation_ = msfb_.GetCurrentState() == RobotMotionState::SquatPreparation;
        msfb_.UpdateCurrentState(RobotMotionState::MimicControl_ChairUp_Mode);
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
            PrepareEntryTransition();
            std::cout << "[MIMIC_CONTROL_CHAIR_UP_MODE] Playing " << policy_->FrameCount()
                      << " embedded ONNX frames from frame 0; transitions allowed after "
                      << (from_squat_preparation_ ? policy_->FrameCount() : kSwitchAfterFrames)
                      << " frames; hold final frame if no request; T MimicControl_ChairDown_Mode / "
                      << "C AmpControl_NiRenWalk_Mode / V AmpControl_Walk2Run_Mode / "
                      << "motion 2 RLControl_Stand_Mode, Z zeropos, R damping\n";
        } catch (const std::exception& error) { Fail(error); }
    }

    void OnExit() override {
        arm_entry_transition_.Cancel();
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
            MatXf output = command_;
            arm_entry_transition_.Apply(output, 1, ri_ptr_->GetInterfaceTimeStamp());
            ri_ptr_->SetJointCommand(output);
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
            if (target == RobotMotionState::MimicControl_ChairDown_Mode) return StateName::kMimicControlChairDownMode;
            if (target == RobotMotionState::AmpControl_GunShootWalk_Mode ||
                target == RobotMotionState::AmpControl_NiRenWalk_Mode ||
                target == RobotMotionState::AmpControl_Walk2Run_Mode) {
                if (RequiresRLControlStandTransition(msfb_.GetCurrentState(), target))
                    return StateName::kRLControlStandMode;
                if (target == RobotMotionState::AmpControl_GunShootWalk_Mode)
                    return StateName::kAmpControlGunShootWalkMode;
                if (target == RobotMotionState::AmpControl_NiRenWalk_Mode)
                    return StateName::kAmpControlNiRenWalkMode;
                return StateName::kAmpControlWalk2RunMode;
            }
            if (target == RobotMotionState::RLControl_Stand_Mode) return StateName::kRLControlStandMode;
        }
        return StateName::kMimicControlChairUpMode;
    }
};
} // namespace humanoid_std
