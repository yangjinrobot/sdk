#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

#ifdef USE_SIMULATION
#include "common_policy_runner_depth_sim.hpp"
#else
#include "common_policy_runner_depth.hpp"
#endif
#include "state_base.h"

namespace humanoid {

class DepthWalkState : public StateBase {
 private:
  RobotBasicState robot_state_;
  std::mutex robot_state_mutex_;
  std::atomic<int> state_run_count_{-1};

  std::shared_ptr<DepthWalkPolicyRunner> depth_policy_;
  UserCommand user_command_{};
  std::mutex user_command_mutex_;

  std::thread policy_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> policy_available_{false};
  bool policy_entered_ = false;
  int filter_index_ = 0;

  void InitializeRobotState() {
    robot_state_.flt_base_acc_mat = Eigen::MatrixXf::Zero(20, 3);
    robot_state_.flt_joint_vel_mat = Eigen::MatrixXf::Zero(20, cp_ptr_->dof_num_);
    robot_state_.flt_base_omega_mat = Eigen::MatrixXf::Zero(20, 3);
  }

  void UpdateRobotObservation() {
    std::lock_guard<std::mutex> lock(robot_state_mutex_);
    robot_state_.base_rpy = ri_ptr_->GetImuRpy();
    robot_state_.base_rot_mat = RpyToRm(robot_state_.base_rpy);
    robot_state_.base_omega = ri_ptr_->GetImuOmega();
    robot_state_.base_acc = ri_ptr_->GetImuAcc();
    robot_state_.joint_pos = ri_ptr_->GetJointPosition();
    robot_state_.joint_vel = ri_ptr_->GetJointVelocity();
    robot_state_.joint_tau = ri_ptr_->GetJointTorque();
    robot_state_.flt_base_acc_mat.row(filter_index_) = robot_state_.base_acc.transpose();
    robot_state_.flt_joint_vel_mat.row(filter_index_) = robot_state_.joint_vel.transpose();
    robot_state_.flt_base_omega_mat.row(filter_index_) = robot_state_.base_omega.transpose();
    filter_index_ = (filter_index_ + 1) % 20;
  }

  void PolicyLoop() {
    constexpr auto kDepthStaleTimeout = std::chrono::milliseconds(200);
    int last_run_count = -1;
    uint64_t last_depth_sequence = 0;
    bool have_depth_sequence = false;
    auto last_new_depth_time = std::chrono::steady_clock::now();
    while (running_) {
      if (state_run_count_ - last_run_count < depth_policy_->decimation_) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        continue;
      }

      UserCommand command;
      {
        std::lock_guard<std::mutex> lock(user_command_mutex_);
        command = user_command_;
      }

      std::vector<float> depth;
      int width = 0;
      int height = 0;
      uint64_t sequence = 0;
      if (!ri_ptr_->GetDepthFrame(depth, width, height, sequence)) {
        policy_available_ = false;
        std::cerr << "[depth_walk] depth source is unavailable" << std::endl;
        break;
      }

      const auto now = std::chrono::steady_clock::now();
      if (!have_depth_sequence || sequence != last_depth_sequence) {
        last_depth_sequence = sequence;
        last_new_depth_time = now;
        have_depth_sequence = true;
      } else if (now - last_new_depth_time >= kDepthStaleTimeout) {
        policy_available_ = false;
        std::cerr << "[depth_walk] no new depth frame for 200 ms; switching to damping"
                  << std::endl;
        break;
      }

      if (!depth_policy_->SetRawDepthFrame(depth, width, height, sequence)) {
        policy_available_ = false;
        std::cerr << "[depth_walk] invalid depth frame; expected 64x36 32FC1 meters"
                  << std::endl;
        break;
      }

      try {
        RobotBasicState robot_snapshot;
        {
          std::lock_guard<std::mutex> lock(robot_state_mutex_);
          robot_snapshot = robot_state_;
        }
        MatXf action = depth_policy_->GetRobotAction(robot_snapshot, command).ConvertToMat();
        MatXf waist = action.block<3, 5>(0, 0);
        MatXf arms = MatXf::Zero(2 * cp_ptr_->arm_dof_num_, 5);
        MatXf legs = MatXf::Zero(2 * cp_ptr_->leg_dof_num_, 5);
        MatXf neck = MatXf::Zero(cp_ptr_->neck_dof_num_, 5);
        arms.block<7, 5>(0, 0) = action.block<7, 5>(3, 0);
        arms.block<7, 5>(7, 0) = action.block<7, 5>(10, 0);
        legs.block<6, 5>(0, 0) = action.block<6, 5>(17, 0);
        legs.block<6, 5>(6, 0) = action.block<6, 5>(23, 0);

        MatXf joint_command(cp_ptr_->dof_num_, 5);
        joint_command << waist, arms, legs, neck;
        ri_ptr_->SetJointCommand(joint_command);
        last_run_count = state_run_count_;
      } catch (const std::exception& e) {
        policy_available_ = false;
        std::cerr << "[depth_walk] inference failed: " << e.what() << std::endl;
        break;
      }
    }
  }

 public:
  DepthWalkState(const RobotName& robot_name, const std::string& state_name,
                 std::shared_ptr<ControllerData> data_ptr)
      : StateBase(robot_name, state_name, data_ptr) {
    depth_policy_ = std::make_shared<DepthWalkPolicyRunner>("depth_walk", robot_name);
    depth_policy_->SetCmdMaxVel(Vec3f(1.0f, 0.5f, 1.0f));
    depth_policy_->DisplayPolicyInfo();
    InitializeRobotState();
  }

  ~DepthWalkState() {
    running_ = false;
    if (policy_thread_.joinable()) {
      policy_thread_.join();
    }
    if (policy_entered_) {
      depth_policy_->OnExit();
    }
  }

  void OnEnter() override {
    state_run_count_ = -1;
    UpdateRobotObservation();

    std::vector<float> depth;
    int width = 0;
    int height = 0;
    uint64_t sequence = 0;
    policy_available_ =
        depth_policy_->IsReady() && ri_ptr_->GetDepthFrame(depth, width, height, sequence) &&
        depth_policy_->SetRawDepthFrame(depth, width, height, sequence);
    if (!policy_available_) {
      std::cerr << "[depth_walk] policy or depth source is not ready; switching to damping"
                << std::endl;
      motion_state_store_->SetState(RobotMotionState::JointDamping);
      return;
    }

    depth_policy_->OnEnter();
    // OnEnter resets history; seed it again with the latest simulator/camera frame.
    depth_policy_->SetRawDepthFrame(depth, width, height, sequence);
    policy_entered_ = true;
    {
      std::lock_guard<std::mutex> lock(user_command_mutex_);
      UserCommand* external = uc_ptr_->GetUserCommand();
      external->target_mode = uint8_t(RobotMotionState::DepthWalkControlMode);
      user_command_ = *external;
    }
    motion_state_store_->SetState(RobotMotionState::DepthWalkControlMode);
    running_ = true;
    policy_thread_ = std::thread(&DepthWalkState::PolicyLoop, this);
  }

  void OnExit() override {
    running_ = false;
    if (policy_thread_.joinable()) {
      policy_thread_.join();
    }
    if (policy_entered_) {
      depth_policy_->OnExit();
      policy_entered_ = false;
    }
    state_run_count_ = -1;
  }

  void Run() override {
    UpdateRobotObservation();
    ++state_run_count_;
    std::lock_guard<std::mutex> lock(user_command_mutex_);
    user_command_ = *uc_ptr_->GetUserCommand();
  }

  bool LoseControlJudge() override {
    std::lock_guard<std::mutex> lock(user_command_mutex_);
    return !policy_available_ ||
           user_command_.target_mode == uint8_t(RobotMotionState::JointDamping);
  }

  StateName GetNextStateName() override {
    std::lock_guard<std::mutex> lock(user_command_mutex_);
    if (!policy_available_ || user_command_.safe_control_mode != 0 ||
        user_command_.target_mode == uint8_t(RobotMotionState::JointDamping)) {
      return StateName::kJointDamping;
    }
    if (user_command_.target_mode == uint8_t(RobotMotionState::RLStandControlMode)) {
      return StateName::kRLStandControl;
    }
    if (user_command_.target_mode == uint8_t(RobotMotionState::GroundStandControlMode)) {
      return StateName::kGroundStandControl;
    }
    if (user_command_.target_mode == uint8_t(RobotMotionState::WholeBodyControlMode)) {
      return StateName::kWholeBodyControl;
    }
    if (user_command_.target_mode == uint8_t(RobotMotionState::ZeroPos)) {
      return StateName::kZeroPos;
    }
    return StateName::kDepthWalkControl;
  }
};

}  // namespace humanoid
