#pragma once

#include <array>
#include <deque>
#include <filesystem>
#include <stdexcept>

#include "policy_runner_base.hpp"

// Runner for policy_stand_0907_dr02_2.onnx.  Its training interface is:
// [omega(3), projected_gravity(3), q(29), dq(29), previous_action(29), 0.3]
// stacked from oldest to newest for six policy frames.
class GroundStandPolicyRunner final : public PolicyRunnerBase {
 private:
  static constexpr int kPolicyDof = 29;
  static constexpr int kSingleObservation = 94;
  static constexpr int kHistory = 6;
  static constexpr int kObservationSize = kSingleObservation * kHistory;
  // host_ground.py gates when real_episode_length_buf > unactuated_time.
  // With unactuated_timesteps=30 and a 20 ms control period, indices 0..30
  // are gated and index 31 is the first policy inference.
  static constexpr int kUnactuatedPolicySteps = 30;

  VecXf kp_, kd_, action_, last_action_, observation_, stacked_observation_;
  std::deque<VecXf> observation_history_;
  std::vector<int> policy_to_robot_;
  std::string policy_path_;
  Ort::SessionOptions session_options_;
  Ort::Session session_{nullptr};
  Ort::MemoryInfo memory_info_{nullptr};
  Ort::Env env_;
  std::vector<Ort::Value> inputs_;
  bool policy_loaded_ = false;
  int policy_step_ = 0;
  const char* input_names_[1] = {"observations"};
  const char* output_names_[1] = {"actions"};

  static void Clip(VecXf& value, float limit) {
    value = value.cwiseMax(-limit).cwiseMin(limit);
  }

  void UpdateObservationHistory() {
    if (static_cast<int>(observation_history_.size()) == kHistory) {
      observation_history_.pop_front();
    }
    observation_history_.push_back(observation_);
    for (int i = 0; i < kHistory; ++i) {
      stacked_observation_.segment(i * kSingleObservation, kSingleObservation) = observation_history_[i];
    }
  }

  RobotAction MakeIncrementalAction(const RobotBasicState& robot) const {
    RobotAction result;
    result.goal_joint_pos = VecXf::Zero(kPolicyDof);
    result.goal_joint_vel = VecXf::Zero(kPolicyDof);
    result.kp = VecXf::Zero(kPolicyDof);
    result.kd = VecXf::Zero(kPolicyDof);
    result.tau_ff = VecXf::Zero(kPolicyDof);
    // Training uses q_target = q_current + 0.3 * action. For a gated zero
    // action this retains the current position and applies only velocity damping.
    for (int policy_index = 0; policy_index < kPolicyDof; ++policy_index) {
      const int robot_index = policy_to_robot_[policy_index];
      result.goal_joint_pos(robot_index) = robot.joint_pos(robot_index) + 0.3f * action_(policy_index);
      result.kp(robot_index) = kp_(policy_index);
      result.kd(robot_index) = kd_(policy_index);
    }
    return result;
  }

 public:
  GroundStandPolicyRunner(const std::string& policy_name, RobotName robot_name)
      : PolicyRunnerBase(policy_name, robot_name), env_(ORT_LOGGING_LEVEL_ERROR, policy_name.c_str()) {
    // Policy order: left leg, right leg, waist, left arm, right arm.
    // Robot command order: waist, left arm, right arm, left leg, right leg.
    policy_to_robot_ = {17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
                        0,  1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11,
                        12, 13, 14, 15, 16};

    kp_.resize(kPolicyDof);
    kd_.resize(kPolicyDof);
    // Gains copied from dr02_pro_2_config_ground.py in policy joint order.
    kp_ << 300, 300, 300, 300, 80, 30, 300, 300, 300, 300, 80, 30,
           200, 200, 200, 100, 100, 100, 100, 30, 30, 30, 100, 100,
           100, 100, 30, 30, 30;
    kp_ = kp_ * 1.1;
    kd_ << 10, 10, 10, 10, 3, 1, 10, 10, 10, 10, 3, 1,
           10, 10, 10, 5, 5, 5, 5, 1, 1, 1, 5, 5, 5, 5, 1, 1, 1;

    decimation_ = 4;  // 200 Hz state loop -> 50 Hz policy, matching training.
    action_.setZero(kPolicyDof);
    last_action_.setZero(kPolicyDof);
    observation_.setZero(kSingleObservation);
    stacked_observation_.setZero(kObservationSize);

    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator,
                                               OrtMemType::OrtMemTypeDefault);
    const auto policy = std::filesystem::path(__FILE__).parent_path() / ".." / "policy" /
                        "policy_stand_0907_dr02_2.onnx";
    try {
      policy_path_ = std::filesystem::canonical(policy).string();
      session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
      policy_loaded_ = true;
      std::cout << "loaded ground stand policy: " << policy_path_ << std::endl;
    } catch (const std::exception& e) {
      policy_path_ = std::filesystem::absolute(policy).string();
      std::cerr << "failed to load ground stand policy " << policy_path_ << ": " << e.what() << std::endl;
    }
  }

  void DisplayPolicyInfo() override {
    std::cout << policy_name_ << " policy: obs=" << kObservationSize << " action=" << kPolicyDof << std::endl;
  }
  bool IsReady() const override { return policy_loaded_; }

  void OnEnter() override {
    action_.setZero();
    last_action_.setZero();
    policy_step_ = 0;
    observation_history_.assign(kHistory, VecXf::Zero(kSingleObservation));
    stacked_observation_.setZero();
  }
  void OnExit() override {}

  RobotAction GetRobotAction(const RobotBasicState& robot, const UserCommand&) override {
    if (!policy_loaded_) throw std::runtime_error("policy is not loaded: " + policy_path_);
    if (robot.joint_pos.size() < kPolicyDof || robot.joint_vel.size() < kPolicyDof) {
      throw std::runtime_error("ground stand policy requires 29 joint observations");
    }

    // Exact host_ground.py / sim2sim gate: policy indices 0 through 30 use
    // zero observations and zero actions; inference begins at index 31.
    const bool is_unactuated = policy_step_ <= kUnactuatedPolicySteps;
    ++policy_step_;
    if (is_unactuated) {
      observation_.setZero();
      action_.setZero();
      last_action_.setZero();
      UpdateObservationHistory();
      return MakeIncrementalAction(robot);
    }

    observation_.setZero();
    observation_.segment<3>(0) = robot.base_omega * 0.25f;
    observation_.segment<3>(3) = robot.base_rot_mat.transpose() * Vec3f(0.f, 0.f, -1.f);
    for (int policy_index = 0; policy_index < kPolicyDof; ++policy_index) {
      const int robot_index = policy_to_robot_[policy_index];
      observation_(6 + policy_index) = robot.joint_pos(robot_index);
      observation_(35 + policy_index) = robot.joint_vel(robot_index) * 0.05f;
    }
    observation_.segment(64, kPolicyDof) = last_action_;
    observation_(93) = 0.3f;
    Clip(observation_, 100.f);
    UpdateObservationHistory();

    const std::array<int64_t, 2> shape{1, kObservationSize};
    auto tensor = Ort::Value::CreateTensor<float>(memory_info_, stacked_observation_.data(),
                                                   stacked_observation_.size(), shape.data(), shape.size());
    inputs_.clear();
    inputs_.push_back(std::move(tensor));
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, inputs_.data(), inputs_.size(),
                                output_names_, 1);
    action_ = Eigen::Map<VecXf>(outputs[0].GetTensorMutableData<float>(), kPolicyDof);
    Clip(action_, 100.f);
    last_action_ = action_;

    return MakeIncrementalAction(robot);
  }
};
