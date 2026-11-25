#include "humanoid/idle_state.hpp"

#include <chrono>
#include <iostream>
#include <thread>

namespace humanoid {

IdleState::IdleState(const RobotName& robot_name, const std::string& state_name,
                     std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr) {
    kp_ = VecXf::Zero(cp_ptr_->dof_num_);
    kd_ = VecXf::Zero(cp_ptr_->dof_num_);
    joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
    if (control_mode_ == ControlMode::kUpperBody) {
        kp_ << cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_, cp_ptr_->leg_kp_, cp_ptr_->leg_kp_,
            cp_ptr_->neck_kp_;
        kd_ << cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_, cp_ptr_->leg_kd_,
            cp_ptr_->neck_kd_;
        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(2) = kd_;
    }
    mark_zero_sub_ = std::make_unique<ChannelUInt64>(
        [this](const std_msgs::msg::UInt64* data) { HandlerMarkZero(data); }, "/JOINTS_MARK_ZERO", 0);
}

IdleState::~IdleState() = default;

void IdleState::OnEnter() {
    motion_state_store_->SetState(RobotMotionState::Idle);
    GetProprioceptiveData();
    RecordJointData();
    last_driver_reset_time_ = ri_ptr_->GetInterfaceTimeStamp();
    std::lock_guard<std::mutex> lock(mark_zero_mutex_);
    pending_mark_zero_mask_ = 0;
    mark_zero_enabled_ = true;
}

void IdleState::OnExit() {
    std::lock_guard<std::mutex> lock(mark_zero_mutex_);
    mark_zero_enabled_ = false;
    pending_mark_zero_mask_ = 0;
}

void IdleState::Run() {
    ProcessMarkZero();
    if (control_mode_ == ControlMode::kUpperBody) {
        GetProprioceptiveData();

        VecXf joint_pos_plan = VecXf::Zero(cp_ptr_->dof_num_);

        joint_pos_plan = joint_pos_init_;
        joint_cmd_.col(1) = joint_pos_plan;

        ri_ptr_->SetJointCommand(joint_cmd_);
    } else {
        // The target hardware continuously reuses the last packet. Clear it
        // in whole-body idle instead of retaining the prior active command.
        ri_ptr_->SetJointCommand(MatXf::Zero(cp_ptr_->dof_num_, 5));
    }
}

bool IdleState::LoseControlJudge() {
    return false;
}

StateName IdleState::GetNextStateName() {
    const auto command_snapshot = uc_ptr_->GetUserCommandSnapshot();

    // Restore the working SDK's Idle recovery: clear driver faults and
    // re-enable joints at most once every 3 seconds. Interface time is seconds.
    if (command_snapshot.safe_control_mode & (1u << kSafeDriverStatus)) {
        const double now = ri_ptr_->GetInterfaceTimeStamp();
        if (now - last_driver_reset_time_ >= kDriverResetIntervalSeconds) {
            std::cout << "[Idle] Driver fault recovery: clear faults and re-enable joints" << std::endl;
            ri_ptr_->ResetJointError();
            last_driver_reset_time_ = now;
        }
    }

    if (command_snapshot.safe_control_mode != 0) {
        std::cout << "safe_control_mode:" << std::dec << command_snapshot.safe_control_mode << std::endl;
        return StateName::kIdle;
    }

    if (command_snapshot.target_mode == RobotMotionState::ZeroPos) {
        return StateName::kZeroPos;
    }

    if (command_snapshot.target_mode == RobotMotionState::AmpZeroPos) {
        return StateName::kAmpZeroPos;
    }

    if (command_snapshot.target_mode == RobotMotionState::AmpWalkingZeroPos) {
        return StateName::kAmpWalkingZeroPos;
    }

    return StateName::kIdle;
}

void IdleState::HandlerMarkZero(const std_msgs::msg::UInt64* data) {
    if (!data)
        return;
    const uint64_t valid_mask =
        cp_ptr_->dof_num_ >= 64 ? ~uint64_t{0} : (uint64_t{1} << cp_ptr_->dof_num_) - 1;
    std::lock_guard<std::mutex> lock(mark_zero_mutex_);
    // Never defer calibration requests received in an active motion state.
    if (mark_zero_enabled_)
        pending_mark_zero_mask_ |= data->data() & valid_mask;
}

void IdleState::ProcessMarkZero() {
    uint64_t mask;
    {
        std::lock_guard<std::mutex> lock(mark_zero_mutex_);
        mask = pending_mark_zero_mask_;
        pending_mark_zero_mask_ = 0;
    }
    if (mask == 0)
        return;

    std::cout << "[Idle] Mark joint zero, mask=" << std::dec << mask << std::endl;
    // Clear the previous command before SetHome. Keep the working SDK's
    // 5 ms delay so the driver can update before restoring position offsets.
    ri_ptr_->SetJointCommand(MatXf::Zero(cp_ptr_->dof_num_, 5));
    ri_ptr_->SetJointZero(mask);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ri_ptr_->InitPositionOffset(mask);
    ri_ptr_->WrapPositionOffset();
    GetProprioceptiveData();
    RecordJointData();
    std::cout << "[Idle] Mark joint zero and position offset update finished, mask=" << std::dec << mask
              << std::endl;
}

void IdleState::GetProprioceptiveData() {
    joint_pos_ = ri_ptr_->GetJointPosition();
    joint_vel_ = ri_ptr_->GetJointVelocity();
    joint_tau_ = ri_ptr_->GetJointTorque();
}

void IdleState::RecordJointData() {
    joint_pos_init_ = joint_pos_;
    joint_vel_init_ = joint_vel_;
}

} // namespace humanoid
