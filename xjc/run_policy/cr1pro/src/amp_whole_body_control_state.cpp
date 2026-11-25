#include "humanoid_pro/amp_whole_body_control_state.hpp"

#include "humanoid_amp_policy_runner.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace humanoid {

AmpWholeBodyControlState::AmpWholeBodyControlState(const RobotName& robot_name, const std::string& state_name,
                                                   std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr) {
    policy_ptr_ = std::make_shared<HumanoidAmpPolicyRunner>("common", robot_name);
    policy_ptr_->SetCmdMaxVel(Vec3f(1.0, 0.5, 1.57));
    policy_ptr_->DisplayPolicyInfo();

    InitializeObservation();
}

AmpWholeBodyControlState::~AmpWholeBodyControlState() {
    start_flag_ = false;
    if (run_policy_thread_.joinable())
        run_policy_thread_.join();
}

void AmpWholeBodyControlState::OnEnter() {
    try {
        state_run_cnt_ = -1;
        policy_available_ = false;
        policy_entered_ = false;
        if (!policy_ptr_)
            throw std::runtime_error("Missing policy");
        UpdateRobotObservation();
        const auto positions = ri_ptr_->GetJointPosition();
        if (positions.size() != cp_ptr_->dof_num_ || !positions.allFinite())
            throw std::runtime_error("AMP whole body requires finite 31-joint feedback");
        Vec7f left = positions.segment<7>(3), right = positions.segment<7>(10);
        const MatXf previous = ri_ptr_->GetJointCommand();
        if (previous.rows() == 31 && previous.cols() == 5 && previous.allFinite()) {
            left = previous.block<7, 1>(3, 1);
            right = previous.block<7, 1>(10, 1);
        }
        policy_ptr_->SetInitialArmTargets(left, right);
        policy_ptr_->OnEnter();
        policy_entered_ = true;
        policy_available_ = policy_ptr_->IsReady();
        if (!policy_available_)
            throw std::runtime_error("Policy is not ready");
        {
            std::lock_guard<std::mutex> lock(user_command_mutex_);
            user_command_ = uc_ptr_->GetUserCommandSnapshot();
        }
        start_flag_ = true;
        motion_state_store_->SetState(RobotMotionState::AmpWholeBodyControlMode);
        run_policy_thread_ = std::thread(&AmpWholeBodyControlState::PolicyRunner, this);

    } catch (const std::exception& error) {
        start_flag_ = false;
        if (run_policy_thread_.joinable())
            run_policy_thread_.join();
        policy_available_ = false;
        motion_state_store_->SetState(RobotMotionState::JointDamping);
        std::cerr << "Policy lifecycle failure: " << error.what() << std::endl;
    } catch (...) {
        start_flag_ = false;
        if (run_policy_thread_.joinable())
            run_policy_thread_.join();
        policy_available_ = false;
        motion_state_store_->SetState(RobotMotionState::JointDamping);
    }
}

void AmpWholeBodyControlState::OnExit() {
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
        if (run_policy_thread_.joinable())
            run_policy_thread_.join();
        policy_available_ = false;
        motion_state_store_->SetState(RobotMotionState::JointDamping);
        std::cerr << "Policy lifecycle failure: " << error.what() << std::endl;
    } catch (...) {
        start_flag_ = false;
        if (run_policy_thread_.joinable())
            run_policy_thread_.join();
        policy_available_ = false;
        motion_state_store_->SetState(RobotMotionState::JointDamping);
    }
}

void AmpWholeBodyControlState::Run() {
    UpdateRobotObservation();

    state_run_cnt_++;
    {
        std::lock_guard<std::mutex> lock(user_command_mutex_);
        user_command_ = uc_ptr_->GetUserCommandSnapshot();
    }
}

bool AmpWholeBodyControlState::LoseControlJudge() {
    if (!policy_available_)
        return true;
    if (user_command_.target_mode == (RobotMotionState::JointDamping))
        return true;
    return false;
}

StateName AmpWholeBodyControlState::GetNextStateName() {
    std::lock_guard<std::mutex> lock(user_command_mutex_);
    if (!policy_available_) {
        return StateName::kJointDamping;
    }
    if (user_command_.safe_control_mode != 0 ||
        user_command_.target_mode == (RobotMotionState::JointDamping)) {
        return StateName::kJointDamping;
    }
    if (user_command_.target_mode == RobotMotionState::AmpZeroPos)
        return StateName::kAmpZeroPos;
    if (user_command_.target_mode == (RobotMotionState::AmpWalkingControl)) {
        return StateName::kAmpWalkingControl;
    }
    return StateName::kAmpWholeBodyControl;
}

void AmpWholeBodyControlState::InitializeObservation() {
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

void AmpWholeBodyControlState::UpdateRobotObservation() {
    std::lock_guard<std::mutex> lock(rbs_mutex_);
    rbs_.base_rpy = ri_ptr_->GetImuRpy();
    rbs_.base_rot_mat = RpyToRm(rbs_.base_rpy);
    rbs_.base_omega = ri_ptr_->GetImuOmega();
    rbs_.base_acc = ri_ptr_->GetImuAcc();
    rbs_.joint_pos = ri_ptr_->GetJointPosition();
    rbs_.joint_vel = ri_ptr_->GetJointVelocity();
    rbs_.joint_tau = ri_ptr_->GetJointTorque();

    rbs_.flt_base_acc_mat.row(observation_history_index_) = rbs_.base_acc.transpose();
    rbs_.flt_joint_vel_mat.row(observation_history_index_) = rbs_.joint_vel.transpose();
    rbs_.flt_base_omega_mat.row(observation_history_index_) = rbs_.base_omega.transpose();

    observation_history_index_ += 1;
    observation_history_index_ = observation_history_index_ % 20;
}

void AmpWholeBodyControlState::PolicyRunner() {
    try {
        int last_policy_tick = -1;
        MatXf res;
        MatXf joint_cmd(cp_ptr_->dof_num_, 5);
        MatXf leg_joint_cmd(2 * cp_ptr_->leg_dof_num_, 5);
        MatXf arm_joint_cmd(2 * cp_ptr_->arm_dof_num_, 5);
        MatXf waist_joint_cmd(cp_ptr_->waist_dof_num_, 5);
        MatXf neck_joint_cmd(cp_ptr_->neck_dof_num_, 5);

        UserCommand command_snapshot;
        RobotBasicState rbs_snapshot;
        while (start_flag_) {
            {
                std::lock_guard<std::mutex> lock(user_command_mutex_);
                command_snapshot = user_command_;
            }

            if (state_run_cnt_ - last_policy_tick >= policy_ptr_->decimation_) {
                {
                    std::lock_guard<std::mutex> lock(rbs_mutex_);
                    rbs_snapshot = rbs_;
                }

                auto ra = policy_ptr_->GetRobotAction(rbs_snapshot, command_snapshot); // actdim(12)*5
                if (!policy_ptr_->IsReady())
                    throw std::runtime_error("policy inference is no longer ready");
                MatXf res = ra.ConvertToMat();
                if (res.rows() != 29 || res.cols() != 5)
                    throw std::runtime_error("PRO policy must produce a 29x5 command");

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
                last_policy_tick = state_run_cnt_;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }

    } catch (const std::exception& error) {
        start_flag_ = false;
        policy_available_ = false;
        std::cerr << state_name_ << " worker failure: " << error.what() << std::endl;
    } catch (...) {
        start_flag_ = false;
        policy_available_ = false;
        std::cerr << state_name_ << " worker failure" << std::endl;
    }
}

} // namespace humanoid
