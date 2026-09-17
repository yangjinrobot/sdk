#ifndef HUMANOID_SHOOT_DOWN_STATE_HPP_
#define HUMANOID_SHOOT_DOWN_STATE_HPP_

#include "shoot_down_policy_runner.hpp"
#include "state_base.h"

#include <chrono>
#include <exception>

namespace humanoid {

class ShootDownState final : public StateBase {
public:
    ShootDownState(const RobotName& robot_name, const std::string& state_name,
                   std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr), policy_(robot_name) {}

    void OnEnter() override {
        failed_ = false;
        reported_finished_ = false;
        run_count_ = 0;
        command_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
        policy_.OnEnter();
        policy_.AlignHeading(ReadRobotState().base_rot_mat);
        msfb_.UpdateCurrentState(RobotMotionState::ShootDown);
        std::cout << "[SHOOT_DOWN] Starting " << policy_.FrameCount() << " frames at 50 Hz" << std::endl;
    }

    void OnExit() override { policy_.OnExit(); }

    void Run() override {
        if (failed_ || uc_ptr_->GetUserCommand()->safe_control_mode != 0 ||
            uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::JointDamping) return;
        // Match training: after the reference ends, its final frame remains
        // the command input and the actor continues closed-loop inference.
        if (run_count_ % policy_.decimation_ == 0) {
            try {
                const auto start = std::chrono::steady_clock::now();
                command_ = policy_.GetRobotAction(ReadRobotState(), *uc_ptr_->GetUserCommand()).ConvertToMat();
                if (std::chrono::steady_clock::now() - start > std::chrono::milliseconds(20))
                    throw std::runtime_error("inference exceeded the 20 ms policy period");
                if (policy_.HoldingFinalFrame() && !reported_finished_) {
                    std::cout << "[SHOOT_DOWN] Holding final reference frame with ONNX inference; "
                                 "press 2 for shoot_up or R for damping" << std::endl;
                    reported_finished_ = true;
                }
            } catch (const std::exception& error) {
                std::cerr << "[SHOOT_DOWN] " << error.what() << "; entering damping" << std::endl;
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
        if (uc_ptr_->GetUserCommand()->target_mode == RobotMotionState::ShootUp)
            return StateName::kShootUp;
        return StateName::kShootDown;
    }

private:
    RobotBasicState ReadRobotState() const {
        RobotBasicState robot;
        robot.base_rpy = ri_ptr_->GetImuRpy();
        robot.base_rot_mat = RpyToRm(robot.base_rpy);
        robot.base_omega = ri_ptr_->GetImuOmega();
        robot.joint_pos = ri_ptr_->GetJointPosition();
        robot.joint_vel = ri_ptr_->GetJointVelocity();
        return robot;
    }

    ShootDownPolicyRunner policy_;
    MatXf command_;
    int run_count_{0};
    bool failed_{false};
    bool reported_finished_{false};
};

}  // namespace humanoid

#endif
