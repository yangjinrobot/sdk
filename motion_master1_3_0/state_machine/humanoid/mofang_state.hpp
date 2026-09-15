#ifndef HUMANOID_MOFANG_STATE_HPP_
#define HUMANOID_MOFANG_STATE_HPP_

#include "mimic_policy_runner.h"
#include "state_base.h"

#include <chrono>
#include <exception>

namespace humanoid {

class MofangState : public StateBase {
public:
    MofangState(const RobotName& robot_name, const std::string& state_name,
                std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr), policy_(robot_name) {
        policy_.DisplayPolicyInfo();
    }

    void OnEnter() override {
        failed_ = false;
        phase_ = Phase::kPlaying;
        run_count_ = 0;
        if (cp_ptr_->dof_num_ != MimicPolicyRunner::kJointCount ||
            ri_ptr_->GetJointPosition().size() != MimicPolicyRunner::kJointCount ||
            ri_ptr_->GetJointVelocity().size() != MimicPolicyRunner::kJointCount) {
            std::cerr << "[MOFANG] Requires exactly 21 robot joints; entering damping" << std::endl;
            failed_ = true;
            return;
        }
        command_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
        policy_.OnEnter();
        RobotBasicState robot = ReadRobotState();
        policy_.AlignHeading(robot.base_rot_mat);
        msfb_.UpdateCurrentState(RobotMotionState::Mofang);
        std::cout << "[MOFANG] Starting NPZ motion directly (" << policy_.FrameCount()
                  << " frames at 50 Hz)" << std::endl;
    }

    void OnExit() override { policy_.OnExit(); }

    void Run() override {
        if (failed_ || uc_ptr_->GetUserCommand()->safe_control_mode != 0 ||
            uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::JointDamping) return;

        if (phase_ == Phase::kPlaying && run_count_ % policy_.decimation_ == 0) {
            try {
                const auto inference_start = std::chrono::steady_clock::now();
                RobotAction action = policy_.GetRobotAction(
                    ReadRobotState(), *uc_ptr_->GetUserCommand());
                if (std::chrono::steady_clock::now() - inference_start >
                    std::chrono::milliseconds(20))
                    throw std::runtime_error("inference exceeded the 20 ms policy period");
                command_ = action.ConvertToMat();
                if (policy_.Finished()) {
                    phase_ = Phase::kHolding;
                    std::cout << "[MOFANG] NPZ motion finished; holding the last target, "
                                 "press C for RL or V for restand" << std::endl;
                }
            } catch (const std::exception& error) {
                std::cerr << "[MOFANG] " << error.what() << "; entering damping" << std::endl;
                failed_ = true;
                return;
            }
        }
        ri_ptr_->SetJointCommand(command_);
        ++run_count_;
    }

    bool LoseControlJudge() override {
        return failed_ || uc_ptr_->GetUserCommand()->safe_control_mode != 0 ||
               uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        if (LoseControlJudge()) return StateName::kJointDamping;
        if (uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::RLControlMode)
            return StateName::kRLControl;
        if (uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::ReStand)
            return StateName::kReStand;
        return StateName::kMofang;
    }

private:
    enum class Phase { kPlaying, kHolding };

    RobotBasicState ReadRobotState() const {
        RobotBasicState robot;
        robot.base_rpy = ri_ptr_->GetImuRpy();
        robot.base_rot_mat = RpyToRm(robot.base_rpy);
        robot.base_omega = ri_ptr_->GetImuOmega();
        robot.joint_pos = ri_ptr_->GetJointPosition();
        robot.joint_vel = ri_ptr_->GetJointVelocity();
        return robot;
    }

    MimicPolicyRunner policy_;
    Phase phase_{Phase::kPlaying};
    MatXf command_;
    int run_count_{0};
    bool failed_{false};
};

}  // namespace humanoid

#endif
