/**
 * @file whole_body_control_state.hpp
 * @brief rl policy runnning state
 * @author deeprobotics
 * @version 1.0
 * @date 2024-05-29
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 */
#pragma once

#include <chrono>

#include "arm_policy_runner.hpp"
#include "state_base.h"

using namespace std::chrono;
namespace humanoid {
class ArmControlState : public StateBase {
private:
    RobotBasicState rbs_;
    int state_run_cnt_;

    std::shared_ptr<PolicyRunnerBase> policy_ptr_;
    std::shared_ptr<ArmPolicyRunner> arm_policy_;
    UserCommand user_command;
    std::mutex user_command_policy_mutex_;

    std::thread arm_policy_thread_;
    bool start_flag_ = true;

    int acc_rot_count = 0;

    void InitRbs() {
        rbs_.flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
        rbs_.flt_joint_vel_mat = Eigen::MatrixXf::Zero(20, cp_ptr_->dof_num_);
        rbs_.flt_base_omega_mat = Eigen::MatrixXf::Zero(20, 3);
    }

    void UpdateRobotObservation() {
        rbs_.base_rpy = ri_ptr_->GetImuRpy();
        rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
        rbs_.base_omega = ri_ptr_->GetImuOmega();
        rbs_.base_acc = ri_ptr_->GetImuAcc();
        rbs_.joint_pos = ri_ptr_->GetJointPosition();
        rbs_.joint_vel = ri_ptr_->GetJointVelocity();
        rbs_.joint_tau = ri_ptr_->GetJointTorque();

        rbs_.flt_base_acc_mat.row(acc_rot_count) = rbs_.base_acc.transpose();
        rbs_.flt_joint_vel_mat.row(acc_rot_count) = rbs_.joint_vel.transpose();
        rbs_.flt_base_omega_mat.row(acc_rot_count) = rbs_.base_omega.transpose();

        acc_rot_count += 1;
        acc_rot_count = acc_rot_count % 20;
    }

    void PolicyRunner() {
        int run_cnt_record = -1;
        MatXf res;
        MatXf joint_cmd(cp_ptr_->dof_num_, 5);
        MatXf leg_joint_cmd(2 * cp_ptr_->leg_dof_num_, 5);
        MatXf arm_joint_cmd(2 * cp_ptr_->arm_dof_num_, 5);
        MatXf waist_joint_cmd(cp_ptr_->waist_dof_num_, 5);
        MatXf neck_joint_cmd(cp_ptr_->neck_dof_num_, 5);

        UserCommand user_command_tmp;
        while (start_flag_) {
            {
                std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
                memcpy(&user_command_tmp, &user_command, sizeof(UserCommand));
            }

            if (state_run_cnt_ - run_cnt_record >= policy_ptr_->decimation_) {

                auto ra = policy_ptr_->GetRobotAction(rbs_, user_command_tmp);  // actdim(12)*5
                MatXf res = ra.ConvertToMat();

                waist_joint_cmd.setZero();
                arm_joint_cmd.setZero();
                neck_joint_cmd.setZero();
                leg_joint_cmd.setZero();


                // 29DOF 策略映射（按 robot_order 前29个关节的顺序）
                // 腰部 3 个关节
                waist_joint_cmd = res.block<3, 5>(0, 0);
                // 左臂 7 个关节
                arm_joint_cmd.block<7, 5>(0, 0) = res.block<7, 5>(3, 0);
                // 右臂 7 个关节
                arm_joint_cmd.block<7, 5>(7, 0) = res.block<7, 5>(10, 0);
                // 左腿 6 个关节
                leg_joint_cmd.block<6, 5>(0, 0) = res.block<6, 5>(17, 0);
                // 右腿 6 个关节
                leg_joint_cmd.block<6, 5>(6, 0) = res.block<6, 5>(23, 0);


                joint_cmd << waist_joint_cmd, arm_joint_cmd, leg_joint_cmd, neck_joint_cmd;

                ri_ptr_->SetJointCommand(joint_cmd);
                run_cnt_record = state_run_cnt_;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

public:
    ArmControlState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr) {
        std::memset(&rbs_, 0, sizeof(rbs_));
        arm_policy_ = std::make_shared<ArmPolicyRunner>("arm_policy", robot_name);
        arm_policy_->SetCmdMaxVel(Vec3f(1.0, 0.5, 1.57));
        arm_policy_->DisplayPolicyInfo();

        std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
        policy_ptr_ = arm_policy_;
        if (!policy_ptr_) {
            std::cerr << "error policy" << std::endl;
            exit(0);
        }
        InitRbs();
    }

    ~ArmControlState() {
        if (arm_policy_thread_.joinable()) {
            arm_policy_thread_.join();
        }
    }

    virtual void OnEnter() {
        state_run_cnt_ = -1;
        UpdateRobotObservation();
        start_flag_ = true;
        {
            std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
            policy_ptr_->OnEnter();
            // 获取外部命令指针
            UserCommand* ext_cmd = uc_ptr_->GetUserCommand();
            // 强制设置为 ArmControlMode
            ext_cmd->target_mode = uint8_t(RobotMotionState::ArmControlMode);
            // 拷贝到本地
            memcpy(&user_command, ext_cmd, sizeof(UserCommand));
        }
        // 修正：应该是 ArmControlMode
        motion_state_store_->SetState(RobotMotionState::ArmControlMode);
        arm_policy_thread_ = std::thread(std::bind(&ArmControlState::PolicyRunner, this));
    }

    virtual void OnExit() {
        start_flag_ = false;
        policy_ptr_->OnExit();
        if (arm_policy_thread_.joinable()) {
            arm_policy_thread_.join();
        }
        state_run_cnt_ = -1;
    }

    virtual void Run() {
        UpdateRobotObservation();

        state_run_cnt_++;
        {
            std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
            memcpy(&user_command, uc_ptr_->GetUserCommand(), sizeof(UserCommand));
        }
    }

    virtual bool LoseControlJudge() {
        if (user_command.target_mode == (RobotMotionState::JointDamping)) return true;
        return false;
    }

    virtual StateName GetNextStateName() {
        std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
        if (user_command.safe_control_mode != 0 || user_command.target_mode == (RobotMotionState::JointDamping)) {
            return StateName::kJointDamping;
        }

        if (user_command.target_mode == uint8_t(RobotMotionState::WholeBodyControlMode)) {
            std::cout << "ArmControl -> WholeBodyControl" << std::endl;
            return StateName::kWholeBodyControl;
        }
        if (user_command.target_mode == uint8_t(RobotMotionState::RunControlMode)) {
            std::cout << "ArmControl -> RunControl" << std::endl;
            return StateName::kRunControl;
        }
        if (user_command.target_mode == uint8_t(RobotMotionState::GroundStandControlMode)) {
            return StateName::kGroundStandControl;
        }

        return StateName::kArmControl;
    }
};
};
