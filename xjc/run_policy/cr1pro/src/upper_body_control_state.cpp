#include "humanoid_pro/upper_body_control_state.hpp"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <cmath>

namespace humanoid {

UpperBodyControlState::UpperBodyControlState(const RobotName& robot_name, const std::string& state_name,
                                             std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr) {
    InitializeObservation();
}

UpperBodyControlState::~UpperBodyControlState() {
    start_flag_ = false;
    if (run_policy_thread_.joinable())
        run_policy_thread_.join();
}

void UpperBodyControlState::OnEnter() {
    worker_failed_ = false;
    state_run_cnt_ = -1;

    UpdateRobotObservation();
    start_flag_ = true;
    {
        std::lock_guard<std::mutex> lock(user_command_mutex_);
        user_command_ = uc_ptr_->GetUserCommandSnapshot();
    }
    motion_state_store_->SetState(RobotMotionState::UpperBodyControlMode);
    run_policy_thread_ = std::thread(&UpperBodyControlState::PolicyRunner, this);
}

void UpperBodyControlState::OnExit() {
    start_flag_ = false;
    if (run_policy_thread_.joinable())
        run_policy_thread_.join();
    state_run_cnt_ = -1;
}

void UpperBodyControlState::Run() {
    UpdateRobotObservation();

    state_run_cnt_++;
    {
        std::lock_guard<std::mutex> lock(user_command_mutex_);
        user_command_ = uc_ptr_->GetUserCommandSnapshot();
    }
}

bool UpperBodyControlState::LoseControlJudge() {
    if (worker_failed_ || user_command_.safe_control_mode != 0)
        return true;
    if (user_command_.target_mode == (RobotMotionState::JointDamping))
        return true;
    return false;
}

StateName UpperBodyControlState::GetNextStateName() {
    if (LoseControlJudge())
        return StateName::kJointDamping;
    if (user_command_.target_mode == (RobotMotionState::ZeroPos))
        return StateName::kZeroPos;
    return StateName::kUpperBodyControl;
}

void UpperBodyControlState::InitializeObservation() {
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

void UpperBodyControlState::UpdateRobotObservation() {
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

void UpperBodyControlState::PolicyRunner() {
    try {
        MatXf joint_cmd(cp_ptr_->dof_num_, 5);
        MatXf leg_joint_cmd = MatXf::Zero(2 * cp_ptr_->leg_dof_num_, 5);
        MatXf neck_joint_cmd = MatXf::Zero(cp_ptr_->neck_dof_num_, 5);
        MatXf arm_joint_cmd(cp_ptr_->arm_dof_num_ * 2, 5);
        MatXf waist_joint_cmd(cp_ptr_->waist_dof_num_, 5);

        float state_enter_time = static_cast<float>(ri_ptr_->GetInterfaceTimeStamp());
        RobotBasicState initial_observation;
        {
            std::lock_guard<std::mutex> lock(rbs_mutex_);
            initial_observation = rbs_;
        }
        VecXf joint_pos_init = initial_observation.joint_pos;
        VecXf joint_vel_init = initial_observation.joint_vel;
        VecXf joint_pos_goal = joint_pos_init;

        constexpr float init_time = 3.0f;
        constexpr float kShoulderLift = 1.1f;
        constexpr float kShoulderOpen = 0.35f;
        constexpr float kElbowBend = 0.35f;

        const int left_arm_start = cp_ptr_->waist_dof_num_;
        const int right_arm_start = cp_ptr_->waist_dof_num_ + cp_ptr_->arm_dof_num_;

        // Upper-body mode only provides a position-control example
        joint_pos_goal(left_arm_start + 0) = joint_pos_init(left_arm_start + 0) - kShoulderLift;
        joint_pos_goal(right_arm_start + 0) = joint_pos_init(right_arm_start + 0) - kShoulderLift;
        joint_pos_goal(left_arm_start + 1) = joint_pos_init(left_arm_start + 1) + kShoulderOpen;
        joint_pos_goal(right_arm_start + 1) = joint_pos_init(right_arm_start + 1) - kShoulderOpen;
        joint_pos_goal(left_arm_start + 3) = joint_pos_init(left_arm_start + 3) + kElbowBend;
        joint_pos_goal(right_arm_start + 3) = joint_pos_init(right_arm_start + 3) + kElbowBend;

        while (start_flag_) {
            VecXf joint_pos_plan = VecXf::Zero(cp_ptr_->dof_num_);
            VecXf joint_vel_plan = VecXf::Zero(cp_ptr_->dof_num_);
            float elapsed_time = static_cast<float>(ri_ptr_->GetInterfaceTimeStamp()) - state_enter_time;

            if (elapsed_time <= init_time) {
                for (int i = 0; i < cp_ptr_->dof_num_; ++i) {
                    joint_pos_plan(i) = GetCubicSplinePos(joint_pos_init(i), joint_vel_init(i),
                                                          joint_pos_goal(i), 0.0f, elapsed_time, init_time);
                    joint_vel_plan(i) = GetCubicSplineVel(joint_pos_init(i), joint_vel_init(i),
                                                          joint_pos_goal(i), 0.0f, elapsed_time, init_time);
                }
            } else {
                joint_pos_plan = joint_pos_goal;
                joint_vel_plan.setZero();
            }

            arm_joint_cmd.setZero();
            waist_joint_cmd.setZero();

            arm_joint_cmd.col(0) = cp_ptr_->arm_kp_.replicate(2, 1);
            arm_joint_cmd.col(1) = joint_pos_plan.segment(cp_ptr_->waist_dof_num_, 2 * cp_ptr_->arm_dof_num_);
            arm_joint_cmd.col(2) = cp_ptr_->arm_kd_.replicate(2, 1);
            arm_joint_cmd.col(3) = joint_vel_plan.segment(cp_ptr_->waist_dof_num_, 2 * cp_ptr_->arm_dof_num_);
            arm_joint_cmd.col(4).setZero();

            waist_joint_cmd.col(0) = cp_ptr_->waist_kp_;
            waist_joint_cmd.col(1) = joint_pos_plan.segment(0, cp_ptr_->waist_dof_num_);
            waist_joint_cmd.col(2) = cp_ptr_->waist_kd_;
            waist_joint_cmd.col(3) = joint_vel_plan.segment(0, cp_ptr_->waist_dof_num_);
            waist_joint_cmd.col(4).setZero();

            // Leg joints are controlled by the low-level policy and do not require assignment
            joint_cmd << waist_joint_cmd, arm_joint_cmd, leg_joint_cmd, neck_joint_cmd;
            ri_ptr_->SetJointCommand(joint_cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

    } catch (const std::exception& error) {
        start_flag_ = false;
        worker_failed_ = true;
        std::cerr << state_name_ << " worker failure: " << error.what() << std::endl;
    } catch (...) {
        start_flag_ = false;
        worker_failed_ = true;
        std::cerr << state_name_ << " worker failure" << std::endl;
    }
}

} // namespace humanoid
