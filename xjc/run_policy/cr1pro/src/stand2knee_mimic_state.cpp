#include "humanoid_pro/stand2knee_mimic_state.hpp"

#include "knee_mimic_policy_runner.h"
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace humanoid {

Stand2KneeMimicState::Stand2KneeMimicState(const RobotName& robot_name, const std::string& state_name,
                                           std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr),
      policy_ptr_(std::make_shared<Stand2KneePolicyRunner>("stand2knee", robot_name, "", data_ptr->udp_node)),
      bounded_command_(*cp_ptr_) {
    if (robot_name != RobotName::CR1PRO || cp_ptr_->dof_num_ != 31)
        throw std::invalid_argument("Imported policies require CR1PRO / 31 DOF");
}

Stand2KneeMimicState::~Stand2KneeMimicState() {
    OnExit();
}

void Stand2KneeMimicState::OnEnter() {
    OnExit();
    policy_available_ = false;
    try {
        state_run_cnt_ = 0;
        rbs_ = ReadObservation();
        user_command_ = uc_ptr_->GetUserCommandSnapshot();
        policy_ptr_->OnEnter();
        policy_entered_ = true;
        if (!policy_ptr_->IsReady() || user_command_.safe_control_mode ||
            user_command_.target_mode == RobotMotionState::JointDamping)
            throw std::runtime_error("Imported policy cannot enter");
        // Initialize reference t=0 and immediately send the first action.
        InferAndSend(rbs_, user_command_);
        motion_state_store_->SetState(RobotMotionState::Stand2KneeMimicMode);
        policy_available_ = true;
        start_flag_ = true;
        run_policy_thread_ = std::thread(&Stand2KneeMimicState::PolicyRunner, this);
    } catch (const std::exception& error) {
        std::cerr << state_name_ << " entry failure: " << error.what() << std::endl;
        OnExit();
        Fail();
        motion_state_store_->SetState(RobotMotionState::JointDamping);
    } catch (...) {
        OnExit();
        Fail();
        motion_state_store_->SetState(RobotMotionState::JointDamping);
    }
}

void Stand2KneeMimicState::OnExit() {
    start_flag_ = false;
    if (run_policy_thread_.joinable())
        run_policy_thread_.join();
    if (policy_entered_) {
        policy_ptr_->OnExit();
        policy_entered_ = false;
    }
    policy_available_ = false;
}

void Stand2KneeMimicState::Run() {
    std::lock_guard<std::mutex> lock(policy_input_mutex_);
    rbs_ = ReadObservation();
    user_command_ = uc_ptr_->GetUserCommandSnapshot();
    ++state_run_cnt_;
}

bool Stand2KneeMimicState::LoseControlJudge() {
    std::lock_guard<std::mutex> lock(policy_input_mutex_);
    return !policy_available_ || user_command_.safe_control_mode ||
           user_command_.target_mode == RobotMotionState::JointDamping;
}

StateName Stand2KneeMimicState::GetNextStateName() {
    std::lock_guard<std::mutex> lock(policy_input_mutex_);
    if (!policy_available_ || user_command_.safe_control_mode ||
        user_command_.target_mode == RobotMotionState::JointDamping)
        return StateName::kJointDamping;
    if (user_command_.target_mode == RobotMotionState::Knee2StandMimicMode)
        return StateName::kKnee2StandMimic;
    return StateName::kStand2KneeMimic;
}

RobotBasicState Stand2KneeMimicState::ReadObservation() const {
    RobotBasicState state{};
    state.base_rpy = ri_ptr_->GetImuRpy();
    state.base_rot_mat = RpyToRm(state.base_rpy);
    state.base_omega = ri_ptr_->GetImuOmega();
    state.base_acc = ri_ptr_->GetImuAcc();
    state.joint_pos = ri_ptr_->GetJointPosition();
    state.joint_vel = ri_ptr_->GetJointVelocity();
    state.joint_tau = ri_ptr_->GetJointTorque();
    return state;
}

void Stand2KneeMimicState::SendDamping() {
    MatXf cmd = MatXf::Zero(cp_ptr_->dof_num_, 5);
    cmd.col(2).setOnes();
    ri_ptr_->SetJointCommand(cmd);
}

void Stand2KneeMimicState::InferAndSend(const RobotBasicState& state, const UserCommand& command) {
    auto action = policy_ptr_->GetRobotAction(state, command);
    if (!policy_ptr_->IsReady())
        throw std::runtime_error("Imported policy inference failed");
    const MatXf output = action.ConvertToMat();
    if (output.rows() != 29 || output.cols() != 5 || !output.allFinite())
        throw std::runtime_error("Imported policy requires finite 29x5 output");
    MatXf full = MatXf::Zero(31, 5);
    full.topRows(29) = output;
    ri_ptr_->SetJointCommand(bounded_command_.Apply(full, state));
}

void Stand2KneeMimicState::Fail() {
    policy_available_ = false;
    start_flag_ = false;
    SendDamping();
}

void Stand2KneeMimicState::PolicyRunner() {
    try {
        int last_policy_tick = 0;
        while (start_flag_) {
            RobotBasicState state;
            UserCommand command;
            int tick;
            {
                std::lock_guard<std::mutex> lock(policy_input_mutex_);
                tick = state_run_cnt_;
                if (tick - last_policy_tick >= policy_ptr_->decimation_) {
                    state = rbs_;
                    command = user_command_;
                }
            }
            if (tick - last_policy_tick >= policy_ptr_->decimation_) {
                InferAndSend(state, command);
                last_policy_tick = tick;
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    } catch (const std::exception& error) {
        std::cerr << state_name_ << " worker failure: " << error.what() << std::endl;
        Fail();
    } catch (...) {
        Fail();
    }
}

} // namespace humanoid
