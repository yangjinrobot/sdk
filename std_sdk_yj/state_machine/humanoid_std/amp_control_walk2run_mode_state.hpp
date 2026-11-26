#ifndef HUMANOID_STD_RUN_STATE_HPP_
#define HUMANOID_STD_RUN_STATE_HPP_

#include "state_base.h"
#include "amp_control_walk2run_mode_policy_runner.h"
#include "arm_entry_transition.hpp"

namespace humanoid_std {
class AmpControlWalk2RunModeState final : public StateBase {
public:
    static constexpr const char* kDefaultPolicyPath =
        "/opt/robot/share/motion_master/policy/amp/1028_run/policy_run_2.onnx";

    // Active running mode retains the established Walk2Run policy and control behavior.
    AmpControlWalk2RunModeState(const RobotName& robot_name, const std::string& state_name,
             std::shared_ptr<ControllerData> data_ptr,
             const std::string& policy_path = kDefaultPolicyPath)
        : StateBase(robot_name, state_name, data_ptr),
          policy_("run", robot_name, policy_path,
                  AmpControlWalk2RunModePolicyRunner::Variant::Run1027) {}

    void OnEnter() override {
        policy_.OnEnter();
        run_count_ = 0;
        failed_ = false;
        joint_cmd_ = ri_ptr_->GetJointCommand();
        if (joint_cmd_.rows() != cp_ptr_->dof_num_ || joint_cmd_.cols() != 5 ||
            !joint_cmd_.allFinite()) {
            std::cerr << "[run] invalid previous joint command; entering damping\n";
            failed_ = true;
            return;
        }
        // Preserve the last command published by RL / RL Stand, including PD.
        arm_entry_transition_.Reset(joint_cmd_.block(1, 0, 8, 5));
        msfb_.UpdateCurrentState(RobotMotionState::AmpControl_Walk2Run_Mode);
        msfb_.UpdateCurrentGait(0);
        std::cout << "[run] policy started at 50 Hz" << std::endl;
    }

    void OnExit() override {
        arm_entry_transition_.Cancel();
        policy_.OnExit();
    }

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
                std::cerr << "[run] " << error.what() << "; entering damping" << std::endl;
                failed_ = true;
                return;
            }
        }
        MatXf command = joint_cmd_;
        arm_entry_transition_.Apply(command, 1);
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
            case RobotMotionState::RLControl_Stand_Mode: return StateName::kRLControlStandMode;
            case RobotMotionState::MjlabControl_Walk2Run_Mode: return StateName::kMjlabControlWalk2RunMode;
            case RobotMotionState::MimicControl_Kneel_Mode: return StateName::kMimicControlKneelMode;
            case RobotMotionState::MimicControl_ChairDown_Mode: return StateName::kMimicControlChairDownMode;
            case RobotMotionState::AmpControl_GunShootWalk_Mode:
                if (RequiresRLControlStandTransition(
                        RobotMotionState::AmpControl_Walk2Run_Mode,
                        RobotMotionState::AmpControl_GunShootWalk_Mode))
                    return StateName::kRLControlStandMode;
                return StateName::kAmpControlGunShootWalkMode;
            case RobotMotionState::AmpControl_NiRenWalk_Mode: return StateName::kAmpControlNiRenWalkMode;
            case RobotMotionState::ZeroPos: return StateName::kZeroPos;
            default: return StateName::kAmpControlWalk2RunMode;
        }
    }

private:
    ArmEntryTransition arm_entry_transition_;
    AmpControlWalk2RunModePolicyRunner policy_;
    MatXf joint_cmd_;
    int run_count_ = 0;
    bool failed_ = false;
};
}  // namespace humanoid_std

#endif
