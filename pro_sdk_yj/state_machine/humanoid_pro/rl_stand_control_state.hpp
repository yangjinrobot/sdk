/**
 * @file rl_stand_control_state.hpp
 * @brief rl policy runnning state
 * @author deeprobotics
 * @version 1.0
 * @date 2024-05-29
 *
 * @copyright Copyright (c) 2024  DeepRobotics
 *
 */
#ifndef HUMANOID_PRO_RL_STAND_CONTROL_STATE_HPP_
#define HUMANOID_PRO_RL_STAND_CONTROL_STATE_HPP_

#include <chrono>
#include <atomic>
#include <stdexcept>

#include "rl_stand_policy_runner.hpp"
#include "state_base.hpp"

using namespace std::chrono;
namespace humanoid_pro {
class RLStandControlState : public StateBase {
private:
    RobotBasicState rbs_{};
    std::mutex rbs_mutex_;
    std::atomic<int> state_run_cnt_{-1};

    std::shared_ptr<PolicyRunnerBase> policy_ptr_;
    std::shared_ptr<RLStandPolicyRunner> rl_stand_policy_;

    UserCommand user_command{};
    std::mutex user_command_policy_mutex_;

    std::thread run_policy_thread_;
    std::atomic<bool> start_flag_{false};
    std::atomic<bool> policy_available_{false};
    bool policy_entered_ = false;
    const RobotMotionState motion_mode_;

    int acc_rot_count = 0;

    void InitRbs() {
        rbs_.base_rpy.setZero();
        rbs_.base_quat.setZero();
        rbs_.base_rot_mat.setIdentity();
        rbs_.base_omega.setZero();
        rbs_.base_acc.setZero();
        rbs_.joint_pos = VecXf::Zero(cp_ptr_->dof_num_);
        rbs_.joint_vel = VecXf::Zero(cp_ptr_->dof_num_);
        rbs_.joint_tau = VecXf::Zero(cp_ptr_->dof_num_);
        rbs_.flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
        rbs_.flt_joint_vel_mat = Eigen::MatrixXf::Zero(20, cp_ptr_->dof_num_);
        rbs_.flt_base_omega_mat = Eigen::MatrixXf::Zero(20, 3);
    }

    void UpdateRobotObservation() {
        std::lock_guard<std::mutex> lock(rbs_mutex_);
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
        try {
        int run_cnt_record = -1;
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
            RobotBasicState rbs_snapshot;
            {
                std::lock_guard<std::mutex> lock(rbs_mutex_);
                rbs_snapshot = rbs_;
            }

            if (state_run_cnt_ - run_cnt_record >= policy_ptr_->decimation_) {
                auto ra = policy_ptr_->GetRobotAction(rbs_snapshot, user_command_tmp);
                if (!policy_ptr_->IsReady())
                    throw std::runtime_error("policy inference is no longer ready");
                MatXf res = ra.ConvertToMat();
                if (res.rows() != 29 || res.cols() != 5)
                    throw std::runtime_error("PRO policy must produce a 29x5 command");

                waist_joint_cmd.setZero();
                arm_joint_cmd.setZero();
                neck_joint_cmd.setZero();
                leg_joint_cmd.setZero();

                // arm_joint_cmd.col(0) = cp_ptr_->arm_kp_.replicate(2, 1);
                // arm_joint_cmd.col(2) = cp_ptr_->arm_kd_.replicate(2, 1);
                // waist_joint_cmd.col(0) = cp_ptr_->waist_kp_;
                // waist_joint_cmd.col(2) = cp_ptr_->waist_kd_;

                // // Only replace joints covered by the policy output; keep PD lock for the rest
                // waist_joint_cmd.block<1, 5>(0, 0) = res.block<1, 5>(0, 0);
                // arm_joint_cmd.block<4, 5>(0, 0) = res.block<4, 5>(1, 0);
                // arm_joint_cmd.block<4, 5>(7, 0) = res.block<4, 5>(5, 0);
                // leg_joint_cmd = res.block<12, 5>(9, 0);

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

        } catch (const std::exception& error) {
            start_flag_ = false;
            policy_available_ = false;
            std::cerr << "[rl_stand_control_state.hpp] worker failure: " << error.what() << std::endl;
        } catch (...) {
            start_flag_ = false;
            policy_available_ = false;
            std::cerr << "[rl_stand_control_state.hpp] worker failure" << std::endl;
        }
    }

protected:
    RLStandControlState(const RobotName& robot_name, const std::string& state_name,
                        std::shared_ptr<ControllerData> data_ptr,
                        std::shared_ptr<RLStandPolicyRunner> policy, RobotMotionState motion_mode)
        : StateBase(robot_name, state_name, data_ptr), motion_mode_(motion_mode) {
        rl_stand_policy_ = std::move(policy);
        rl_stand_policy_->SetCmdMaxVel(Vec3f(0.8, 0.5, 0.5));
        rl_stand_policy_->DisplayPolicyInfo();

        std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
        policy_ptr_ = rl_stand_policy_;
        if (!policy_ptr_) {
            std::cerr << "error policy" << std::endl;
            exit(0);
        }
        InitRbs();
    }

public:
    RLStandControlState(const RobotName& robot_name, const std::string& state_name,
                        std::shared_ptr<ControllerData> data_ptr)
        : RLStandControlState(robot_name, state_name, data_ptr,
              std::make_shared<RLStandPolicyRunner>("rl_stand", robot_name), RLControl_Stand_Mode) {}

    ~RLStandControlState() override {
        start_flag_ = false;
        if (run_policy_thread_.joinable()) run_policy_thread_.join();
    }

    void OnEnter() override {
        try {
        state_run_cnt_ = -1;
        policy_available_ = false;
        policy_entered_ = false;
        if (!policy_ptr_) throw std::runtime_error("Missing RL stand policy");
        UpdateRobotObservation();
        if (rbs_.joint_pos.size() != cp_ptr_->dof_num_ || !rbs_.joint_pos.allFinite())
            throw std::runtime_error("RL stand requires finite 31-joint feedback");
        Vec7f initial_left_arm = rbs_.joint_pos.segment<7>(3);
        Vec7f initial_right_arm = rbs_.joint_pos.segment<7>(10);
        const MatXf last_joint_cmd = ri_ptr_->GetJointCommand();
        if (motion_mode_ != RLControl_StandSalute_Mode && motion_mode_ != RLControl_StandSaluteLeftArm_Mode &&
            motion_mode_ != RLControl_StandPlease_Mode && motion_mode_ != RLControl_StandTalking_Mode &&
            motion_mode_ != RLControl_StandPlease2_Mode &&
            last_joint_cmd.rows() == 31 &&
            last_joint_cmd.cols() == 5 && last_joint_cmd.allFinite()) {
            initial_left_arm = last_joint_cmd.block<7, 1>(3, 1);
            initial_right_arm = last_joint_cmd.block<7, 1>(10, 1);
        }
        rl_stand_policy_->SetInitialArmTargets(initial_left_arm, initial_right_arm);

        policy_ptr_->OnEnter();
        policy_entered_ = true;
        policy_available_ = policy_ptr_->IsReady();
        if (!policy_available_) throw std::runtime_error("RL stand policy is not ready");
        {
            std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
            user_command = uc_ptr_->GetUserCommandSnapshot();
        }
        start_flag_ = true;
        motion_state_store_->SetState(motion_mode_);
        run_policy_thread_ = std::thread(std::bind(&RLStandControlState::PolicyRunner, this));

        } catch (const std::exception& error) {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
            std::cerr << "Policy lifecycle failure: " << error.what() << std::endl;
        } catch (...) {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
        }
    }

    void OnExit() override {
        try {
        start_flag_ = false;
        if (run_policy_thread_.joinable()) {
            run_policy_thread_.join();
        }
        if (policy_entered_) {
            policy_ptr_->OnExit();
            policy_entered_ = false;
        }
        state_run_cnt_ = -1;

        } catch (const std::exception& error) {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
            std::cerr << "Policy lifecycle failure: " << error.what() << std::endl;
        } catch (...) {
            start_flag_ = false;
            if (run_policy_thread_.joinable()) run_policy_thread_.join();
            policy_available_ = false;
            motion_state_store_->SetState(RobotMotionState::JointDamping);
        }
    }

    void Run() override {
        UpdateRobotObservation();

        state_run_cnt_++;
        {
            std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
            user_command = uc_ptr_->GetUserCommandSnapshot();
        }
    }

    bool LoseControlJudge() override {
        if (!policy_available_) return true;
        if (user_command.target_mode == (RobotMotionState::JointDamping)) return true;
        return false;
    }

    StateName GetNextStateName() override {
        std::lock_guard<std::mutex> lock(user_command_policy_mutex_);
        if (!policy_available_) {
            return StateName::kJointDamping;
        }
        if (user_command.safe_control_mode != 0 || user_command.target_mode == (RobotMotionState::JointDamping)) {
            return StateName::kJointDamping;
        }

        if (user_command.target_mode == RobotMotionState::RLControl_StandSalute_Mode)
            return StateName::kRLStandSalute;
        if (user_command.target_mode == RobotMotionState::RLControl_StandSaluteLeftArm_Mode)
            return StateName::kRLStandSaluteLeftArm;
        if (user_command.target_mode == RobotMotionState::RLControl_StandPlease_Mode)
            return StateName::kRLStandPlease;
        if (user_command.target_mode == RobotMotionState::RLControl_StandTalking_Mode)
            return StateName::kRLStandTalking;
        if (user_command.target_mode == RobotMotionState::RLControl_StandPlease2_Mode)
            return StateName::kRLStandPlease2;
        if (user_command.target_mode == RobotMotionState::AmpControl_Walk2Run_Mode)
            return StateName::kRLControl;
        if (user_command.target_mode == (RobotMotionState::AmpControl_Walk_Mode)) {
            return StateName::kAmpWalkingControl;
        }

        return static_cast<StateName>(motion_mode_);
    }
};
}  // namespace humanoid_pro

#endif
