#pragma once

#include "state_base.h"
#include "mimic_control_chair_down_mode_policy_runner.hpp"
#include "arm_entry_transition.hpp"
#include <chrono>

namespace humanoid_std {

// Enter from RL, nirenwalk or completed mimic_control_chair_up_mode standing control.
// Play once, then keep
// evaluating the final reference with live feedback until Y, Z or R is requested.
class MimicControlChairDownModeState : public StateBase {
    std::shared_ptr<MimicControlChairDownModePolicyRunner> policy_;
    RobotBasicState robot_{};
    MatXf command_;
    ArmEntryTransition arm_entry_transition_;
    int ticks_ = 0;
    bool failed_ = false;

    void ReadObservation() {
        robot_.joint_pos = ri_ptr_->GetJointPosition();
        robot_.joint_vel = ri_ptr_->GetJointVelocity();
        robot_.base_rpy = ri_ptr_->GetImuRpy();
        robot_.base_rot_mat = RpyToRm(robot_.base_rpy);
        robot_.base_omega = ri_ptr_->GetImuOmega();
        if (robot_.joint_pos.size() != 21 || robot_.joint_vel.size() != 21 ||
            !robot_.joint_pos.allFinite() || !robot_.joint_vel.allFinite() ||
            !robot_.base_rot_mat.allFinite() || !robot_.base_omega.allFinite())
            throw std::runtime_error("Invalid mimic_control_chair_down_mode feedback");
    }

    void Fail(const std::exception& error) {
        failed_ = true;
        std::cerr << "[MIMIC_CONTROL_CHAIR_DOWN_MODE] " << error.what() << "; entering damping\n";
    }

    void PrepareEntryTransition() {
        arm_entry_transition_.Cancel();
        const MatXf previous = ri_ptr_->GetJointCommand();
        if (previous.rows() != cp_ptr_->dof_num_ || previous.cols() != 5 || !previous.allFinite()) {
            std::cerr << "[MIMIC_CONTROL_CHAIR_DOWN_MODE] unavailable previous command; skipping arm entry blend\n";
            return;
        }
        arm_entry_transition_.Reset(previous.block(1, 0, 8, 5), ri_ptr_->GetInterfaceTimeStamp());
    }

public:
    MimicControlChairDownModeState(const RobotName& robot, const std::string& name,
                       std::shared_ptr<ControllerData> data)
        : StateBase(robot, name, data) {
        if (robot != RobotName::CR1STANDARD || cp_ptr_->dof_num_ != 21)
            throw std::invalid_argument("Stand2sit mimic requires CR1STANDARD / 21 DOF");
        policy_ = std::make_shared<MimicControlChairDownModePolicyRunner>(robot);
        policy_->DisplayPolicyInfo();
    }

    void OnEnter() override {
        msfb_.UpdateCurrentState(RobotMotionState::MimicControl_ChairDown_Mode);
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
            std::cout << "[MIMIC_CONTROL_CHAIR_DOWN_MODE] Playing " << policy_->FrameCount()
                      << " embedded ONNX frames from frame 0; hold final frame; Y rises after completion, Z zeropos, R damping\n";
        } catch (const std::exception& error) { Fail(error); }
    }

    void OnExit() override {
        arm_entry_transition_.Cancel();
        policy_->OnExit();
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
                    throw std::runtime_error("Stand2sit inference exceeded 20 ms");
#endif
            }
            ticks_ = (ticks_ + 1) % policy_->decimation_;
            MatXf output = command_;
            arm_entry_transition_.Apply(output, 1, ri_ptr_->GetInterfaceTimeStamp());
            ri_ptr_->SetJointCommand(output);
        } catch (const std::exception& error) { Fail(error); }
    }

    bool LoseControlJudge() override {
        const auto& user = *uc_ptr_->GetUserCommand();
        return failed_ || user.safe_control_mode != 0 || user.target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        if (LoseControlJudge()) return StateName::kJointDamping;
        if (uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::ZeroPos) return StateName::kZeroPos;
        // Apply the final action for its full 20 ms before accepting a pending Y.
        if (uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::MimicControl_ChairUp_Mode &&
            policy_->IsMotionFinished() && ticks_ == 0) return StateName::kMimicControlChairUpMode;
        return StateName::kMimicControlChairDownMode;
    }
};
} // namespace humanoid_std
