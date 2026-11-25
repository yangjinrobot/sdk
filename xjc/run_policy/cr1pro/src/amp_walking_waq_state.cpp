#include "humanoid_pro/amp_walking_waq_state.hpp"

#include "amp_walking_waq_policy_runner.hpp"
#include <chrono>
#include <iostream>
#include <stdexcept>

namespace humanoid {

AmpWalkingWAQState::AmpWalkingWAQState(const RobotName& robot_name, const std::string& state_name,
                                       std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr),
      policy_ptr_(std::make_shared<AmpWalkingWAQPolicyRunner>("amp_walk_to_run", robot_name)),
      bounded_command_(*cp_ptr_) {}

AmpWalkingWAQState::~AmpWalkingWAQState() {
    OnExit();
}

void AmpWalkingWAQState::OnEnter() {
    OnExit();
    try {
        state_run_cnt_ = 0;
        rbs_ = ReadObservation();
        if (rbs_.joint_pos.size() != 31 || !rbs_.joint_pos.allFinite())
            throw std::runtime_error("AMP Walking WAQ state requires finite 31-joint feedback");
        policy_ptr_->OnEnter();
        policy_entered_ = true;
        policy_available_ = policy_ptr_->IsReady();
        if (!policy_available_)
            throw std::runtime_error("AMP Walking WAQ policy is not ready");
        user_command_ = uc_ptr_->GetUserCommandSnapshot();
        if (user_command_.safe_control_mode != 0 ||
            user_command_.target_mode == RobotMotionState::JointDamping)
            throw std::runtime_error("AMP walk-to-run entry blocked by damping/safety request");
        // Fill the five-frame walk-to-run history from this observation,
        // then immediately apply the first action and hold it for 20 ms.
        InferAndSend(rbs_, user_command_);
        motion_state_store_->SetState(RobotMotionState::AmpWalkingWAQ);
        start_flag_ = true;
        run_policy_thread_ = std::thread(&AmpWalkingWAQState::PolicyRunner, this);
    } catch (const std::exception& error) {
        std::cerr << "AMP Walking WAQ entry failure: " << error.what() << std::endl;
        OnExit();
        policy_available_ = false;
        motion_state_store_->SetState(RobotMotionState::JointDamping);
    } catch (...) {
        OnExit();
        policy_available_ = false;
        motion_state_store_->SetState(RobotMotionState::JointDamping);
    }
}

void AmpWalkingWAQState::OnExit() {
    start_flag_ = false;
    if (run_policy_thread_.joinable())
        run_policy_thread_.join();
    if (policy_entered_) {
        policy_ptr_->OnExit();
        policy_entered_ = false;
    }
    policy_available_ = false;
}

void AmpWalkingWAQState::Run() {
    {
        std::lock_guard<std::mutex> lock(policy_input_mutex_);
        rbs_ = ReadObservation();
        user_command_ = uc_ptr_->GetUserCommandSnapshot();
        ++state_run_cnt_;
    }
}

bool AmpWalkingWAQState::LoseControlJudge() {
    std::lock_guard<std::mutex> lock(policy_input_mutex_);
    return !policy_available_ || user_command_.safe_control_mode != 0 ||
           user_command_.target_mode == RobotMotionState::JointDamping;
}

StateName AmpWalkingWAQState::GetNextStateName() {
    std::lock_guard<std::mutex> lock(policy_input_mutex_);
    if (!policy_available_ || user_command_.safe_control_mode != 0 ||
        user_command_.target_mode == RobotMotionState::JointDamping)
        return StateName::kJointDamping;
    if (user_command_.target_mode == RobotMotionState::RLStandControlMode)
        return StateName::kRLStandControl;
    if (user_command_.target_mode == RobotMotionState::RLStandSaluteMode)
        return StateName::kRLStandSalute;
    if (user_command_.target_mode == RobotMotionState::RLStandSaluteLeftArmMode)
        return StateName::kRLStandSaluteLeftArm;
    if (user_command_.target_mode == RobotMotionState::AmpWalkingControl)
        return StateName::kAmpWalkingControl;
    return StateName::kAmpWalkingWAQ;
}

RobotBasicState AmpWalkingWAQState::ReadObservation() {
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

void AmpWalkingWAQState::InferAndSend(const RobotBasicState& state, const UserCommand& command) {
    auto action = policy_ptr_->GetRobotAction(state, command);
    if (!policy_ptr_->IsReady())
        throw std::runtime_error("AMP Walking WAQ inference failed");
    const MatXf output = action.ConvertToMat();
    if (output.rows() != 29 || output.cols() != 5 || !output.allFinite())
        throw std::runtime_error("AMP Walking WAQ requires finite 29x5 output");
    MatXf full = MatXf::Zero(cp_ptr_->dof_num_, 5);
    full.topRows(29) = output;
    // Running can request virtual ankle targets beyond SDK position limits.
    // Bound the packet while preserving the requested effort within motor limits.
    ri_ptr_->SetJointCommand(bounded_command_.Apply(full, state));
}

void AmpWalkingWAQState::PolicyRunner() {
    try {
        int last_policy_tick = 0; // OnEnter already sent the t=0 action.
        while (start_flag_) {
            RobotBasicState state;
            UserCommand command;
            int tick;
            {
                std::lock_guard<std::mutex> lock(policy_input_mutex_);
                tick = state_run_cnt_.load();
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
        std::cerr << "AMP Walking WAQ worker failure: " << error.what() << std::endl;
        policy_available_ = false;
        start_flag_ = false;
    } catch (...) {
        policy_available_ = false;
        start_flag_ = false;
    }
}

} // namespace humanoid
