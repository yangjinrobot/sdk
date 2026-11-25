#include "humanoid_pro/rl_stand_salute_left_arm_state.hpp"

#include "rl_stand_salute_policy_runner.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace humanoid {

RLStandSaluteLeftArmState::RLStandSaluteLeftArmState(const RobotName& robot_name,
                                                     const std::string& state_name,
                                                     std::shared_ptr<ControllerData> data_ptr)
    : RLStandSaluteLeftArmState(robot_name, state_name, std::move(data_ptr),
                                std::make_shared<RLStandSalutePolicyRunner>("rl_stand_salute", robot_name)) {}

RLStandSaluteLeftArmState::RLStandSaluteLeftArmState(const RobotName& robot_name,
                                                     const std::string& state_name,
                                                     std::shared_ptr<ControllerData> data_ptr,
                                                     std::shared_ptr<RLStandSalutePolicyRunner> policy_ptr)
    : StateBase(robot_name, state_name, data_ptr), policy_ptr_(std::move(policy_ptr)) {
    if (!policy_ptr_)
        throw std::invalid_argument("Missing RL stand salute policy");
    policy_ptr_->SetCmdMaxVel(Vec3f(0.8, 0.5, 0.5));
    policy_ptr_->DisplayPolicyInfo();
    InitializeObservation();
}

RLStandSaluteLeftArmState::~RLStandSaluteLeftArmState() {
    start_flag_ = false;
    if (run_policy_thread_.joinable())
        run_policy_thread_.join();
}

void RLStandSaluteLeftArmState::OnEnter() {
    walking_requested_ = false;
    gesture_transition_ = false;
    policy_ptr_->SetLeftArmMotionEnabled(true);
    try {
        state_run_cnt_ = -1;
        policy_available_ = false;
        policy_entered_ = false;
        if (!policy_ptr_)
            throw std::runtime_error("Missing RL stand policy");
        UpdateRobotObservation();
        if (rbs_.joint_pos.size() != cp_ptr_->dof_num_ || !rbs_.joint_pos.allFinite())
            throw std::runtime_error("RL stand requires finite 31-joint feedback");
        Vec7f initial_left_arm = rbs_.joint_pos.segment<7>(3);
        Vec7f initial_right_arm = rbs_.joint_pos.segment<7>(10);
        policy_ptr_->SetInitialArmTargets(initial_left_arm, initial_right_arm);

        policy_ptr_->OnEnter();
        policy_entered_ = true;
        policy_available_ = policy_ptr_->IsReady();
        if (!policy_available_)
            throw std::runtime_error("RL stand policy is not ready");
        {
            std::lock_guard<std::mutex> lock(user_command_mutex_);
            user_command_ = uc_ptr_->GetUserCommandSnapshot();
        }
        start_flag_ = true;
        motion_state_store_->SetState(RobotMotionState::RLStandSaluteLeftArmMode);
        run_policy_thread_ = std::thread(&RLStandSaluteLeftArmState::PolicyRunner, this);

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

void RLStandSaluteLeftArmState::OnExit() {
    policy_ptr_->PreservePolicyOnExit(gesture_transition_);
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
    walking_requested_ = false;
    gesture_transition_ = false;
}

void RLStandSaluteLeftArmState::Run() {
    UpdateRobotObservation();

    state_run_cnt_++;
    {
        std::lock_guard<std::mutex> lock(user_command_mutex_);
        user_command_ = uc_ptr_->GetUserCommandSnapshot();
    }
}

bool RLStandSaluteLeftArmState::LoseControlJudge() {
    if (!policy_available_)
        return true;
    if (user_command_.target_mode == (RobotMotionState::JointDamping))
        return true;
    return false;
}

StateName RLStandSaluteLeftArmState::GetNextStateName() {
    std::lock_guard<std::mutex> lock(user_command_mutex_);
    gesture_transition_ = false;
    if (!policy_available_ || user_command_.safe_control_mode != 0 ||
        user_command_.target_mode == RobotMotionState::JointDamping)
        return StateName::kJointDamping;
    // RL Control can take over even while the arm is lowering.
    if (user_command_.target_mode == RobotMotionState::RLControlMode)
        return StateName::kRLControl;
    if (user_command_.target_mode == RobotMotionState::AmpWalkingControl) {
        walking_requested_ = true;
        policy_ptr_->RequestArmLowering();
    }
    // Keep standing until the arm trajectory and measured posture agree.
    if (walking_requested_)
        return policy_ptr_->IsArmLoweringComplete() ? StateName::kAmpWalkingControl
                                                    : StateName::kRLStandSaluteLeftArm;
    if (user_command_.target_mode == RobotMotionState::AmpMoe15Dof)
        return StateName::kAmpMoe15Dof;
    if (user_command_.target_mode == RobotMotionState::AmpWalkingWAQ)
        return StateName::kAmpWalkingWAQ;
    if (user_command_.target_mode == RobotMotionState::RLStandSaluteMode) {
        gesture_transition_ = true;
        return StateName::kRLStandSalute;
    }
    return StateName::kRLStandSaluteLeftArm;
}

void RLStandSaluteLeftArmState::InitializeObservation() {
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

void RLStandSaluteLeftArmState::UpdateRobotObservation() {
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

void RLStandSaluteLeftArmState::PolicyRunner() {
    try {
        int last_policy_tick = -1;
        MatXf joint_cmd(cp_ptr_->dof_num_, 5);
        MatXf leg_joint_cmd(2 * cp_ptr_->leg_dof_num_, 5);
        MatXf arm_joint_cmd(2 * cp_ptr_->arm_dof_num_, 5);
        MatXf waist_joint_cmd(cp_ptr_->waist_dof_num_, 5);
        MatXf neck_joint_cmd(cp_ptr_->neck_dof_num_, 5);

        UserCommand command_snapshot;
        while (start_flag_) {
            {
                std::lock_guard<std::mutex> lock(user_command_mutex_);
                command_snapshot = user_command_;
            }
            RobotBasicState rbs_snapshot;
            {
                std::lock_guard<std::mutex> lock(rbs_mutex_);
                rbs_snapshot = rbs_;
            }

            if (state_run_cnt_ - last_policy_tick >= policy_ptr_->decimation_) {
                auto ra = policy_ptr_->GetRobotAction(rbs_snapshot, command_snapshot);
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
