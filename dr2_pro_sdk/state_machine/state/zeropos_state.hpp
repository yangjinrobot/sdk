/**
 * @file ZeroPosState.hpp
 * @brief
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
class ZeroPosState : public StateBase {
private:
    float state_enter_time_, run_time_;
    VecXf joint_pos_init_, joint_vel_init_, joint_pos_, joint_vel_;
    VecXf kp_, kd_, joint_pos_goal_;
    MatXf joint_cmd_;

    void GetRobotJointValue() {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        joint_pos_ = ri_ptr_->GetJointPosition();
        joint_vel_ = ri_ptr_->GetJointVelocity();
    }

    void RecordJointData() {
        state_enter_time_ = run_time_;
        joint_pos_init_ = joint_pos_;
        joint_vel_init_ = joint_vel_;
    }

public:
    ZeroPosState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr) {
        joint_pos_goal_ = VecXf::Zero(cp_ptr_->dof_num_);
        joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
        kp_ = VecXf::Zero(cp_ptr_->dof_num_);
        kd_ = VecXf::Zero(cp_ptr_->dof_num_);

        if (robot_name_ == RobotName::DR2PRO) {
            kp_ << cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_, cp_ptr_->leg_kp_, cp_ptr_->leg_kp_,
                cp_ptr_->neck_kp_;
            kd_ << cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_, cp_ptr_->leg_kd_,
                cp_ptr_->neck_kd_;
               
                // // qizi_1
                joint_pos_goal_ <<  0.0,   // waist_yaw
                                    0.0,   // waist_roll
                                    0.0,   // waist_pitch
                                    // 左臂（7个）
                                    0.0,   // left_shoulder_pitch
                                    0.3,  // left_shoulder_roll
                                    0.0,   // left_shoulder_yaw
                                    1.4,   // left_elbow_pitch
                                    0.0,   // left_wrist_yaw
                                    0.0,   // left_wrist_pitch
                                    0.0,   // left_wrist_roll
                                    // 右臂（7个）
                                    0.0,   // right_shoulder_pitch
                                    -0.3, // right_shoulder_roll
                                    0.0,   // right_shoulder_yaw
                                    1.4,   // right_elbow_pitch
                                    0.0,   // right_wrist_yaw
                                    0.0,   // right_wrist_pitch
                                    0.0,   // right_wrist_roll
                                    // 左腿（6个）
                                    -0.2,  // left_hip_pitch
                                    0.0,   // left_hip_roll
                                    0.0,   // left_hip_yaw
                                    0.4,   // left_knee_pitch
                                    -0.2,  // left_ankle_pitch
                                    0.0,   // left_ankle_roll
                                    // 右腿（6个）
                                    -0.2,  // right_hip_pitch
                                    0.0,   // right_hip_roll
                                    0.0,   // right_hip_yaw
                                    0.4,   // right_knee_pitch
                                    -0.2,  // right_ankle_pitch
                                    0.0;   // right_ankle_roll

                // // qizi_1
                // joint_pos_goal_ <<  0.0,   // waist_yaw
                //                     0.0,   // waist_roll
                //                     0.0,   // waist_pitch
                //                     // 左臂（7个）
                //                     -0.5915,   // left_shoulder_pitch
                //                     0.2953,  // left_shoulder_roll
                //                     -1.5331,   // left_shoulder_yaw
                //                     0.5487,   // left_elbow_pitch
                //                     0.7796,   // left_wrist_yaw
                //                     -0.5925,   // left_wrist_pitch
                //                     -0.2632,   // left_wrist_roll
                //                     // 右臂（7个）
                //                     -1.2687,   // right_shoulder_pitch
                //                     -0.5547, // right_shoulder_roll
                //                     1.7069,   // right_shoulder_yaw
                //                     -0.193,   // right_elbow_pitch
                //                     -1.2758,   // right_wrist_yaw
                //                     -0.1508,   // right_wrist_pitch
                //                     0.1075,   // right_wrist_roll
                //                     // 左腿（6个）
                //                     -0.15,  // left_hip_pitch
                //                     0.0,   // left_hip_roll
                //                     0.0,   // left_hip_yaw
                //                     0.3,   // left_knee_pitch
                //                     -0.15,  // left_ankle_pitch
                //                     0.0,   // left_ankle_roll
                //                     // 右腿（6个）
                //                     -0.15,  // right_hip_pitch
                //                     0.0,   // right_hip_roll
                //                     0.0,   // right_hip_yaw
                //                     0.3,   // right_knee_pitch
                //                     -0.15,  // right_ankle_pitch
                //                     0.0;   // right_ankle_roll

                // qizi_2
                // joint_pos_goal_ <<  0.0,   // waist_yaw
                //                     0.0,   // waist_roll
                //                     0.0,   // waist_pitch
                //                     // 左臂（7个）
                //                     -0.6786,   // left_shoulder_pitch (更新)
                //                     0.4466,    // left_shoulder_roll (更新)
                //                     -1.2867,   // left_shoulder_yaw (更新)
                //                     -0.0168,   // left_elbow_pitch (更新)
                //                     0.7798,    // left_wrist_yaw (更新)
                //                     -0.1354,   // left_wrist_pitch (更新)
                //                     -0.0457,   // left_wrist_roll (更新)
                //                     // 右臂（7个）
                //                     -1.5797,   // right_shoulder_pitch (更新)
                //                     -0.5517,   // right_shoulder_roll (更新)
                //                     1.5231,    // right_shoulder_yaw (更新)
                //                     -0.1677,   // right_elbow_pitch (更新)
                //                     -1.587,    // right_wrist_yaw (更新)
                //                     0.0504,    // right_wrist_pitch (更新)
                //                     0.3227,    // right_wrist_roll (更新)
                //                     // 左腿（6个）
                //                     -0.15,  // left_hip_pitch
                //                     0.0,   // left_hip_roll
                //                     0.0,   // left_hip_yaw
                //                     0.3,   // left_knee_pitch
                //                     -0.15,  // left_ankle_pitch
                //                     0.0,   // left_ankle_roll
                //                     // 右腿（6个）
                //                     -0.15,  // right_hip_pitch
                //                     0.0,   // right_hip_roll
                //                     0.0,   // right_hip_yaw
                //                     0.3,   // right_knee_pitch
                //                     -0.15,  // right_ankle_pitch
                //                     0.0;   // right_ankle_roll
        }
    }

    ~ZeroPosState() {}

    virtual void OnEnter() {
        GetRobotJointValue();
        RecordJointData();
        motion_state_store_->SetState(RobotMotionState::ZeroPos);
    };

    virtual void OnExit() {}

    virtual void Run() {
        GetRobotJointValue();

        VecXf joint_pos_plan = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf joint_vel_plan = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf joint_tor_plan = VecXf::Zero(cp_ptr_->dof_num_);

        float init_time = 3.;
        if (run_time_ - state_enter_time_ <= init_time) {
            for (int i = 0; i < joint_pos_.rows(); ++i) {
                joint_pos_plan(i) = GetCubicSplinePos(joint_pos_init_(i), joint_vel_init_(i), joint_pos_goal_(i), 0,
                                                      run_time_ - state_enter_time_, init_time);
                joint_vel_plan(i) = GetCubicSplineVel(joint_pos_init_(i), joint_vel_init_(i), joint_pos_goal_(i), 0,
                                                      run_time_ - state_enter_time_, init_time);
            }
        } else {
            for (int i = 0; i < joint_pos_.rows(); ++i) {
                joint_pos_plan(i) = joint_pos_goal_(i);
                joint_vel_plan.setZero();
            }
        }

        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(2) = kd_;

        joint_cmd_.col(1) = joint_pos_plan;
        joint_cmd_.col(3) = joint_vel_plan;
        joint_cmd_.col(4) = joint_tor_plan;
        ri_ptr_->SetJointCommand(joint_cmd_);
    }

    virtual bool LoseControlJudge() {
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) {
            return true;
        }
        return false;
    }

    virtual StateName GetNextStateName() {
        if (uc_ptr_->GetUserCommand()->safe_control_mode != 0 ||
            uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::JointDamping)) {
            std::cout << "safe_control_mode:" << std::dec << uc_ptr_->GetUserCommand()->safe_control_mode << std::endl;
            return StateName::kJointDamping;
        }

        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::WholeBodyControlMode)) {
            std::cout << "go to rl control" << std::endl;
            return StateName::kWholeBodyControl;
        }
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::UpperBodyControlMode)) {
            std::cout << "go to upper body control" << std::endl;
            return StateName::kUpperBodyControl;
        }
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::RunControlMode)) {
            std::cout << "go to run control" << std::endl;
            return StateName::kRunControl;
        }
        if (uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::ArmControlMode)) {
            std::cout << "go to arm control" << std::endl;
            return StateName::kArmControl;
        }
        if (control_mode_ == ControlMode::kWholeBody &&
            uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::RLStandControlMode)) {
            std::cout << "go to RL stand control" << std::endl;
            return StateName::kRLStandControl;
        }
        if (control_mode_ == ControlMode::kWholeBody &&
            uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::DepthWalkControlMode)) {
            std::cout << "go to depth walk control" << std::endl;
            return StateName::kDepthWalkControl;
        }
        if (control_mode_ == ControlMode::kWholeBody &&
            uc_ptr_->GetUserCommand()->target_mode == uint8_t(RobotMotionState::GroundStandControlMode)) {
            std::cout << "go to ground stand 0907 control" << std::endl;
            return StateName::kGroundStandControl;
        }
        return StateName::kZeroPos;
    }
};

};
