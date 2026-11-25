#include "humanoid_pro/rl_control_state.hpp"

#include "amp_policy_runner.h"
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace humanoid {

RLControlState::RLControlState(const RobotName& robot_name, const std::string& state_name,
                               std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr),
      policy_ptr_(std::make_shared<AmpPolicyRunner>("amp", robot_name)), bounded_command_(*cp_ptr_) {
    if (robot_name != RobotName::CR1PRO || cp_ptr_->dof_num_ != 31)
        throw std::invalid_argument("Imported policies require CR1PRO / 31 DOF");
    policy_ptr_->SetCmdMaxVel(Vec3f(3.0f, 1.0f, 1.2f));
}

RLControlState::~RLControlState() {
    OnExit();
}

void RLControlState::OnEnter() {
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
        motion_state_store_->SetState(RobotMotionState::RLControlMode);
        policy_available_ = true;
        start_flag_ = true;
        run_policy_thread_ = std::thread(&RLControlState::PolicyRunner, this);
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

void RLControlState::OnExit() {
    start_flag_ = false;
    if (run_policy_thread_.joinable())
        run_policy_thread_.join();
    if (policy_entered_) {
        policy_ptr_->OnExit();
        policy_entered_ = false;
    }
    policy_available_ = false;
}

void RLControlState::Run() {
    std::lock_guard<std::mutex> lock(policy_input_mutex_);
    rbs_ = ReadObservation();
    user_command_ = uc_ptr_->GetUserCommandSnapshot();
    ++state_run_cnt_;
}

bool RLControlState::LoseControlJudge() {
    std::lock_guard<std::mutex> lock(policy_input_mutex_);
    return !policy_available_ || user_command_.safe_control_mode ||
           user_command_.target_mode == RobotMotionState::JointDamping;
}

StateName RLControlState::GetNextStateName() {
    std::lock_guard<std::mutex> lock(policy_input_mutex_);
    if (!policy_available_ || user_command_.safe_control_mode ||
        user_command_.target_mode == RobotMotionState::JointDamping)
        return StateName::kJointDamping;
    if (user_command_.target_mode == RobotMotionState::RLControlMimic12Mode)
        return StateName::kRLMimic12Control;
    if (user_command_.target_mode == RobotMotionState::RLStandControlMode)
        return StateName::kRLStandControl;
    if (user_command_.target_mode == RobotMotionState::AmpWalkingControl)
        return StateName::kAmpWalkingControl;
    if (user_command_.target_mode == RobotMotionState::RLStandSaluteMode)
        return StateName::kRLStandSalute;
    if (user_command_.target_mode == RobotMotionState::RLStandSaluteLeftArmMode)
        return StateName::kRLStandSaluteLeftArm;
    return StateName::kRLControl;
}

RobotBasicState RLControlState::ReadObservation() const {
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

void RLControlState::SendDamping() {
    MatXf cmd = MatXf::Zero(cp_ptr_->dof_num_, 5);
    cmd.col(2).setOnes();
    ri_ptr_->SetJointCommand(cmd);
}

void RLControlState::InferAndSend(const RobotBasicState& state, const UserCommand& command) {
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

void RLControlState::Fail() {
    policy_available_ = false;
    start_flag_ = false;
    SendDamping();
}

void RLControlState::PolicyRunner() {
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
