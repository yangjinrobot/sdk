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
#pragma once
#include "state_base.h"

namespace humanoid {
class IdleState : public StateBase {
private:
    VecXf joint_pos_init_, joint_vel_init_, kp_, kd_,joint_pos_, joint_vel_, joint_tau_;
    MatXf joint_cmd_;
  
    void GetProprioceptiveData() {
        joint_pos_ = ri_ptr_->GetJointPosition();
        joint_vel_ = ri_ptr_->GetJointVelocity();
        joint_tau_ = ri_ptr_->GetJointTorque();
    }

    void RecordJointData() {
        joint_pos_init_ = joint_pos_;
        joint_vel_init_ = joint_vel_;
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
      }
  ~IdleState() {}

    virtual void OnEnter() {
        motion_state_store_->SetState(RobotMotionState::Idle);
        GetProprioceptiveData();
        RecordJointData();
    };

    virtual void OnExit() { }

    virtual void Run() {
        if(control_mode_ == ControlMode::kUpperBody)
        {
            GetProprioceptiveData();

            VecXf joint_pos_plan = VecXf::Zero(cp_ptr_->dof_num_);

            joint_pos_plan = joint_pos_init_;
            joint_cmd_.col(1) = joint_pos_plan;

            ri_ptr_->SetJointCommand(joint_cmd_);
        }
    }

    virtual bool LoseControlJudge() { return false; }

    virtual StateName GetNextStateName() {

        if (uc_ptr_->GetUserCommand()->safe_control_mode != 0) {
            std::cout << "safe_control_mode:" << std::dec << uc_ptr_->GetUserCommand()->safe_control_mode << std::endl;
            return StateName::kIdle;
        }

        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::ZeroPos)) {
            return StateName::kZeroPos;
        }
        if (control_mode_ == ControlMode::kWholeBody &&
            uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::GroundStandControlMode)) {
            return StateName::kGroundStandControl;
        }

        return StateName::kIdle;
    }
};

};
