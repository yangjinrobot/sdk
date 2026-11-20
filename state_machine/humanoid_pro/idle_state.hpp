/**
 * @file idle_state.hpp
 * @brief robot need to confirm sensor input while in idle state
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-12-12
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#ifndef HUMANOID_PRO_IDLE_STATE_HPP_
#define HUMANOID_PRO_IDLE_STATE_HPP_

#include "state_base.hpp"
#include "drdds/core/drdds_core.h"
#include <chrono>
#include <cstdint>
#include <thread>
#include <functional>

namespace humanoid_pro {
class IdleState : public StateBase {
private:
    VecXf init_joint_pos_, init_joint_vel_, kp_, kd_,joint_pos_, joint_vel_, joint_tau_;
    MatXf joint_cmd_;
    static constexpr double kDriverResetIntervalSeconds = 3.0;
    double time_record_ = 0.0;
    std::mutex mark_zero_mutex_;
    uint64_t MarkZeroMask = 0;
    std::unique_ptr<ChannelUInt64> mark_zero_sub_;

    void HandlerMarkZero(const std_msgs::msg::UInt64* message) {
        std::lock_guard<std::mutex> lock(mark_zero_mutex_);
        MarkZeroMask = message->data();
        std::cout << "[Idle] HandlerMarkZero: " << MarkZeroMask << std::endl;
    }

    void ProcessMarkZero() {
        std::lock_guard<std::mutex> lock(mark_zero_mutex_);
        const uint64_t mask = MarkZeroMask;
        if (mask == 0) return;
        ri_ptr_->SetJointZero(mask);
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
        ri_ptr_->InitPositionOffset(mask);
        ri_ptr_->WrapPositionOffset();
        MarkZeroMask = 0;
        GetProprioceptiveData();
        RecordJointData();
        std::cout << "[Idle] Joint zero command applied, mask: " << mask << std::endl;
    }
  
    void GetProprioceptiveData() {
        joint_pos_ = ri_ptr_->GetJointPosition();
        joint_vel_ = ri_ptr_->GetJointVelocity();
        joint_tau_ = ri_ptr_->GetJointTorque();
    }

    void RecordJointData() {
        init_joint_pos_ = joint_pos_;
        init_joint_vel_ = joint_vel_;
    }
public:
    IdleState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr) {
        kp_ = VecXf::Zero(cp_ptr_->dof_num_);
        kd_ = VecXf::Zero(cp_ptr_->dof_num_);
        joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
        if(control_mode_ == ControlMode::kUpperBody)
        {
            kp_<<cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_, cp_ptr_->leg_kp_, cp_ptr_->leg_kp_, cp_ptr_->neck_kp_;
            kd_<<cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_, cp_ptr_->leg_kd_, cp_ptr_->neck_kd_;
            joint_cmd_.col(0) = kp_;
            joint_cmd_.col(2) = kd_;
        }
        mark_zero_sub_ = std::make_unique<ChannelUInt64>(
            std::bind(&IdleState::HandlerMarkZero, this, std::placeholders::_1),
            "/JOINTS_MARK_ZERO", 0);
      }
  ~IdleState() override {
      OnExit();
      mark_zero_sub_.reset();
  }

    void OnEnter() override {
        std::lock_guard<std::mutex> lock(mark_zero_mutex_);
        MarkZeroMask = 0;
        motion_state_store_->SetState(RobotMotionState::Idle);
        GetProprioceptiveData();
        RecordJointData();
        time_record_ = ri_ptr_->GetInterfaceTimeStamp();
    }

    void OnExit() override {
        std::lock_guard<std::mutex> lock(mark_zero_mutex_);
        MarkZeroMask = 0;
    }

    void Run() override {
        GetProprioceptiveData();
        ProcessMarkZero();
        if(control_mode_ == ControlMode::kUpperBody)
        {
            VecXf joint_pos_plan = VecXf::Zero(cp_ptr_->dof_num_);

            joint_pos_plan = init_joint_pos_;
            joint_cmd_.col(1) = joint_pos_plan;

            ri_ptr_->SetJointCommand(joint_cmd_);
        } else {
            // The target hardware continuously reuses the last packet. Clear it
            // in whole-body idle instead of retaining the prior active command.
            ri_ptr_->SetJointCommand(MatXf::Zero(cp_ptr_->dof_num_, 5));
        }
    }

    bool LoseControlJudge() override { return false; }

    StateName GetNextStateName() override {
        const auto command_snapshot = uc_ptr_->GetUserCommandSnapshot();

        // Restore the working SDK's Idle recovery: clear driver faults and
        // re-enable joints at most once every 3 seconds. Interface time is seconds.
        if (command_snapshot.safe_control_mode & (1u << kSafeDriverStatus)) {
            const double now = ri_ptr_->GetInterfaceTimeStamp();
            if (now - time_record_ >= kDriverResetIntervalSeconds) {
                std::cout << "[Idle] Driver fault recovery: clear faults and re-enable joints" << std::endl;
                ri_ptr_->ResetJointError();
                time_record_ = now;
            }
        }

        if (command_snapshot.safe_control_mode != 0) {
            std::cout << "safe_control_mode:" << std::dec << command_snapshot.safe_control_mode << std::endl;
            return StateName::kIdle;
        }

        if (command_snapshot.target_mode == RobotMotionState::ZeroPos) {
            return StateName::kZeroPos;
        }

        if (command_snapshot.target_mode == RobotMotionState::AmpZeroPos) {
            return StateName::kAmpZeroPos;
        }

        if (command_snapshot.target_mode == RobotMotionState::AmpWalkingZeroPos) {
            return StateName::kAmpWalkingZeroPos;
        }
        if (control_mode_ == ControlMode::kWholeBody &&
            command_snapshot.target_mode == RobotMotionState::SquatPreparation) {
            return StateName::kSquatPreparation;
        }

        return StateName::kIdle;
    }
};

}  // namespace humanoid_pro

#endif
