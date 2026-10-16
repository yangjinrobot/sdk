#pragma once

#include <atomic>
#include <chrono>

#include "ground_stand_policy_runner.hpp"
#include "state_base.h"

namespace humanoid {
class GroundStandControlState final : public StateBase {
 private:
  RobotBasicState robot_state_;
  UserCommand user_command_{};
  std::mutex robot_mutex_, command_mutex_;
  std::shared_ptr<GroundStandPolicyRunner> policy_;
  std::thread policy_thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> run_count_{-1};
  std::atomic<bool> policy_available_{false};

  void UpdateObservation() {
    std::lock_guard<std::mutex> lock(robot_mutex_);
    robot_state_.base_rpy = ri_ptr_->GetImuRpy();
    robot_state_.base_rot_mat = RpyToRm(robot_state_.base_rpy);
    robot_state_.base_omega = ri_ptr_->GetImuOmega();
    robot_state_.joint_pos = ri_ptr_->GetJointPosition();
    robot_state_.joint_vel = ri_ptr_->GetJointVelocity();
  }

  void PolicyLoop() {
    int last_count = -1;
    while (running_) {
      if (run_count_ - last_count >= policy_->decimation_) {
        RobotBasicState robot;
        UserCommand command;
        { std::lock_guard<std::mutex> lock(robot_mutex_); robot = robot_state_; }
        { std::lock_guard<std::mutex> lock(command_mutex_); command = user_command_; }
        try {
          const MatXf policy_command = policy_->GetRobotAction(robot, command).ConvertToMat();
          MatXf full_command = MatXf::Zero(cp_ptr_->dof_num_, 5);
          full_command.block(0, 0, 29, 5) = policy_command;
          ri_ptr_->SetJointCommand(full_command);
          last_count = run_count_;
        } catch (const std::exception& e) {
          std::cerr << "ground stand inference failed: " << e.what() << std::endl;
          policy_available_ = false;
          running_ = false;
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

 public:
  GroundStandControlState(const RobotName& robot_name, const std::string& state_name,
                          std::shared_ptr<ControllerData> data)
      : StateBase(robot_name, state_name, data),
        policy_(std::make_shared<GroundStandPolicyRunner>("ground_stand_0907", robot_name)) {
    policy_->DisplayPolicyInfo();
  }

  ~GroundStandControlState() { OnExit(); }

  void OnEnter() override {
    policy_available_ = policy_->IsReady();
    if (!policy_available_) {
      std::cerr << "ground stand policy is unavailable; switching to damping" << std::endl;
      motion_state_store_->SetState(RobotMotionState::JointDamping);
      return;
    }
    UpdateObservation();
    policy_->OnEnter();
    run_count_ = -1;
    { std::lock_guard<std::mutex> lock(command_mutex_); user_command_ = *uc_ptr_->GetUserCommand(); }
    uc_ptr_->GetUserCommand()->target_mode = uint8_t(RobotMotionState::GroundStandControlMode);
    motion_state_store_->SetState(RobotMotionState::GroundStandControlMode);
    running_ = true;
    policy_thread_ = std::thread(&GroundStandControlState::PolicyLoop, this);
  }

  void OnExit() override {
    running_ = false;
    if (policy_thread_.joinable()) policy_thread_.join();
    policy_->OnExit();
  }

  void Run() override {
    UpdateObservation();
    { std::lock_guard<std::mutex> lock(command_mutex_); user_command_ = *uc_ptr_->GetUserCommand(); }
    ++run_count_;
  }

  bool LoseControlJudge() override {
    return !policy_available_ || user_command_.target_mode == uint8_t(RobotMotionState::JointDamping);
  }

  StateName GetNextStateName() override {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!policy_available_ || user_command_.safe_control_mode != 0 ||
        user_command_.target_mode == uint8_t(RobotMotionState::JointDamping)) return StateName::kJointDamping;
    if (user_command_.target_mode == uint8_t(RobotMotionState::ZeroPos)) return StateName::kZeroPos;
    return StateName::kGroundStandControl;
  }
};
}  // namespace humanoid
