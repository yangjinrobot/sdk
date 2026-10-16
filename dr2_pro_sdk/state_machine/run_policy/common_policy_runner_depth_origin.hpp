/**
 * @file common_policy_runner_depth.hpp
 * @brief Real-robot depth encoder + actor runner used by the depth_walk state.
 *
 * Depth images are supplied by DepthWalkState from RobotInterface's
 * /DEPTH_FRAME subscription. This runner does not capture simulator depth.
 */
#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <onnxruntime_cxx_api.h>

#include "policy_runner_base.hpp"

class DepthWalkPolicyRunner : public PolicyRunnerBase {
 private:
  static constexpr int kRawDepthHeight = 36;
  static constexpr int kRawDepthWidth = 64;
  // Previous crop_region: (6, 12, 16, 16)
  static constexpr int kCropUp = 6;
  static constexpr int kCropDown = 12;
  static constexpr int kCropLeft = 16;
  static constexpr int kCropRight = 16;

  // Current crop_region: (18, 0, 16, 16)
  // static constexpr int kCropUp = 18;
  // static constexpr int kCropDown = 0;
  // static constexpr int kCropLeft = 16;
  // static constexpr int kCropRight = 16;
  static constexpr int kEncoderHeight = 18;
  static constexpr int kEncoderWidth = 32;
  static constexpr int kDepthHistoryLength = 37;
  static constexpr int kFrameStack = 8;
  static constexpr int kActionDim = 29;
  static constexpr int kProprioFrameDim = 96;
  static constexpr int kProprioDim = kFrameStack * kProprioFrameDim;
  static constexpr bool kPrintJointTargets = true;  // 打印关节信息
  static constexpr int kPrintEveryInference = 100;  // About 2 s at 50 Hz.
  inline static constexpr std::array<int, 8> kDepthIndices = {1, 6, 11, 16, 21, 26, 31, 36};
  inline static constexpr std::array<const char*, kActionDim> kOutputJointNames = {
      "waist_yaw", "waist_roll", "waist_pitch",
      "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw",
      "left_elbow_pitch", "left_wrist_yaw", "left_wrist_pitch", "left_wrist_roll",
      "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
      "right_elbow_pitch", "right_wrist_yaw", "right_wrist_pitch", "right_wrist_roll",
      "left_hip_pitch", "left_hip_roll", "left_hip_yaw",
      "left_knee_pitch", "left_ankle_pitch", "left_ankle_roll",
      "right_hip_pitch", "right_hip_roll", "right_hip_yaw",
      "right_knee_pitch", "right_ankle_pitch", "right_ankle_roll"};

  VecXf kp_;
  VecXf kd_;
  VecXf default_pos_;
  Vec3f max_cmd_vel_;
  std::vector<int> robot_to_policy_idx_;
  std::vector<int> policy_to_output_idx_;

  std::array<std::deque<VecXf>, 6> term_history_;
  std::array<int, 6> term_dims_ = {3, 3, 3, kActionDim, kActionDim, kActionDim};
  std::deque<cv::Mat> depth_history_;
  uint64_t last_depth_sequence_ = 0;
  bool history_initialized_ = false;

  VecXf action_;
  VecXf last_action_;
  float action_scale_ = 0.25f;
  float omega_scale_ = 0.25f;
  float joint_velocity_scale_ = 0.05f;
  Vec3f command_scale_ = Vec3f::Ones();
  float clip_observations_ = 10.0f;

  Ort::Env env_;
  Ort::SessionOptions session_options_;
  Ort::MemoryInfo memory_info_{nullptr};
  Ort::Session actor_session_{nullptr};
  Ort::Session depth_session_{nullptr};
  std::string actor_path_ = "not loaded";
  std::string depth_path_ = "not loaded";
  std::string actor_input_name_;
  std::string actor_output_name_;
  std::string depth_input_name_;
  std::string depth_output_name_;
  int latent_dim_ = 0;
  bool ready_ = false;
  int joint_target_print_counter_ = 0;
  std::chrono::steady_clock::time_point last_command_print_time_{};

  void PrintJointTargets(const VecXf& targets) {
    if (!kPrintJointTargets || ++joint_target_print_counter_ < kPrintEveryInference) {
      return;
    }
    joint_target_print_counter_ = 0;

    constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
    std::ostringstream output;
    output << std::fixed << std::setprecision(2)
           << "[depth_walk] goal joint positions (deg): ";
    for (int i = 0; i < kActionDim; ++i) {
      output << kOutputJointNames[i] << ':' << targets(i) * kRadToDeg;
      if (i + 1 != kActionDim) {
        output << ' ';
      }
    }
    std::cout << output.str() << std::endl;
  }

  static std::vector<int> GeneratePermutation(const std::vector<std::string>& from,
                                              const std::vector<std::string>& to) {
    std::unordered_map<std::string, int> index;
    for (int i = 0; i < static_cast<int>(from.size()); ++i) {
      index[from[i]] = i;
    }
    std::vector<int> result;
    result.reserve(to.size());
    for (const auto& name : to) {
      const auto it = index.find(name);
      if (it == index.end()) {
        throw std::runtime_error("joint not found while building depth_walk mapping: " + name);
      }
      result.push_back(it->second);
    }
    return result;
  }

  static bool ShapeMatches(const std::vector<int64_t>& actual,
                           const std::vector<int64_t>& expected) {
    if (actual.size() != expected.size()) {
      return false;
    }
    for (size_t i = 0; i < actual.size(); ++i) {
      if (actual[i] > 0 && actual[i] != expected[i]) {
        return false;
      }
    }
    return true;
  }

  static cv::Mat PreprocessDepth(const std::vector<float>& raw, int width, int height) {
    if (width != kRawDepthWidth || height != kRawDepthHeight ||
        raw.size() != static_cast<size_t>(width * height)) {
      return {};
    }
    cv::Mat raw_mat(height, width, CV_32F, const_cast<float*>(raw.data()));
    const cv::Rect roi(kCropLeft, kCropUp, width - kCropLeft - kCropRight,
                       height - kCropUp - kCropDown);
    cv::Mat cropped = raw_mat(roi).clone();
    cv::patchNaNs(cropped, 2.5);
    cv::GaussianBlur(cropped, cropped, cv::Size(3, 3), 1.0);
    cv::max(cropped, 0.0, cropped);
    cv::min(cropped, 2.5, cropped);
    cropped *= 0.4f;
    return cropped;
  }

  void AppendTerms(const std::array<VecXf, 6>& terms) {
    if (!history_initialized_) {
      for (size_t term = 0; term < terms.size(); ++term) {
        term_history_[term].clear();
        for (int frame = 0; frame < kFrameStack; ++frame) {
          term_history_[term].push_back(terms[term]);
        }
      }
      history_initialized_ = true;
      return;
    }
    for (size_t term = 0; term < terms.size(); ++term) {
      term_history_[term].pop_front();
      term_history_[term].push_back(terms[term]);
    }
  }

  VecXf FlattenTerms() const {
    VecXf flattened(kProprioDim);
    int offset = 0;
    // Isaac observation history is term-major:
    // ang_vel[8], gravity[8], command[8], joint_pos[8], joint_vel[8], action[8].
    for (const auto& history : term_history_) {
      for (const auto& frame : history) {
        flattened.segment(offset, frame.size()) = frame;
        offset += frame.size();
      }
    }
    return flattened;
  }

  void InitializeModels() {
    try {
      namespace fs = std::filesystem;
      const fs::path policy_dir = fs::path(__FILE__).parent_path() / ".." / "policy";
      actor_path_ = fs::canonical(policy_dir / "actor_08061.onnx").string();
      depth_path_ = fs::canonical(policy_dir / "0-depth_encoder_08061.onnx").string();
      actor_session_ = Ort::Session(env_, actor_path_.c_str(), session_options_);
      depth_session_ = Ort::Session(env_, depth_path_.c_str(), session_options_);

      Ort::AllocatorWithDefaultOptions allocator;
      actor_input_name_ = actor_session_.GetInputNameAllocated(0, allocator).get();
      actor_output_name_ = actor_session_.GetOutputNameAllocated(0, allocator).get();
      depth_input_name_ = depth_session_.GetInputNameAllocated(0, allocator).get();
      depth_output_name_ = depth_session_.GetOutputNameAllocated(0, allocator).get();

      const auto depth_input_shape =
          depth_session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      const auto depth_output_shape =
          depth_session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      const auto actor_input_shape =
          actor_session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
      const auto actor_output_shape =
          actor_session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

      if (!ShapeMatches(depth_input_shape, {1, 8, kEncoderHeight, kEncoderWidth}) ||
          depth_output_shape.size() != 2 || depth_output_shape[1] <= 0) {
        throw std::runtime_error("unexpected depth encoder input/output shape");
      }
      latent_dim_ = static_cast<int>(depth_output_shape[1]);
      if (!ShapeMatches(actor_input_shape, {1, kProprioDim + latent_dim_}) ||
          !ShapeMatches(actor_output_shape, {1, kActionDim})) {
        throw std::runtime_error("unexpected actor input/output shape");
      }
      ready_ = true;
    } catch (const std::exception& e) {
      std::cerr << "[depth_walk] failed to load ONNX models: " << e.what() << std::endl;
      ready_ = false;
    }
  }

 public:
  DepthWalkPolicyRunner(const std::string& policy_name, RobotName robot_name)
      : PolicyRunnerBase(policy_name, robot_name),
        env_(ORT_LOGGING_LEVEL_ERROR, policy_name.c_str()),
        memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)) {
    decimation_ = 20;
    // sim2sim ramps commands at 2.0, 2.0 and 3.0 units/s. At the
    // 20 ms policy period this is 0.04, 0.04 and 0.06 per inference.
    vel_delta_const_ << 0.04f, 0.04f, 0.06f;
    max_cmd_vel_.setZero();

    kp_.resize(kActionDim);
    kp_ << 300, 300, 200, 300, 300, 200, 300, 300, 300, 300, 300, 100, 100,
        80, 80, 100, 100, 30, 30, 100, 100, 100, 100, 30, 30, 30, 30, 30, 30;
    kp_ = kp_ * 0.8;      // 修改

    kd_.resize(kActionDim);
    kd_ << 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 5, 5, 3, 3, 5, 5,
        1, 1, 5, 5, 5, 5, 1, 1, 1, 1, 1, 1;
    default_pos_.resize(kActionDim);
    default_pos_ << -0.15, -0.15,  // left/right hip pitch
                    0.0,          // waist yaw
                    0.0, 0.0,     // left/right hip roll
                    0.0,          // waist roll
                    0.0, 0.0,     // left/right hip yaw
                    0.0,          // waist pitch
                    0.3, 0.3,     // left/right knee pitch
                    0.0, 0.0,     // left/right shoulder pitch
                    -0.15, -0.15,  // left/right ankle pitch
                    0.2, -0.2,    // left/right shoulder roll
                    0.0, 0.0,     // left/right ankle roll
                    0.0, 0.0,     // left/right shoulder yaw
                    1.4, 1.4,     // left/right elbow pitch
                    0.0, 0.0,     // left/right wrist yaw
                    0.0, 0.0,     // left/right wrist pitch
                    0.0, 0.0;     // left/right wrist roll

    const std::vector<std::string> robot_order = {
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint", "left_wrist_yaw_joint", "left_wrist_pitch_joint",
        "left_wrist_roll_joint", "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint", "right_elbow_pitch_joint", "right_wrist_yaw_joint",
        "right_wrist_pitch_joint", "right_wrist_roll_joint", "left_hip_pitch_joint",
        "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_pitch_joint",
        "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint",
        "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_pitch_joint",
        "right_ankle_pitch_joint", "right_ankle_roll_joint"};
    const std::vector<std::string> policy_order = {
        "left_hip_pitch_joint", "right_hip_pitch_joint",
        "waist_yaw_joint",
        "left_hip_roll_joint", "right_hip_roll_joint",
        "waist_roll_joint",
        "left_hip_yaw_joint", "right_hip_yaw_joint",
         "waist_pitch_joint",
        "left_knee_pitch_joint", "right_knee_pitch_joint",
        "left_shoulder_pitch_joint", "right_shoulder_pitch_joint",
        "left_ankle_pitch_joint", "right_ankle_pitch_joint",
        "left_shoulder_roll_joint", "right_shoulder_roll_joint",
        "left_ankle_roll_joint", "right_ankle_roll_joint",
        "left_shoulder_yaw_joint", "right_shoulder_yaw_joint",
        "left_elbow_pitch_joint", "right_elbow_pitch_joint",
        "left_wrist_yaw_joint", "right_wrist_yaw_joint",
        "left_wrist_pitch_joint", "right_wrist_pitch_joint",
        "left_wrist_roll_joint", "right_wrist_roll_joint"};
    robot_to_policy_idx_ = GeneratePermutation(robot_order, policy_order);
    policy_to_output_idx_ = GeneratePermutation(policy_order, robot_order);

    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    InitializeModels();
  }

  bool IsReady() const override { return ready_; }

  void SetCmdMaxVel(const Vec3f& velocity) { max_cmd_vel_ = velocity.cwiseMax(0.0f); }

  void DisplayPolicyInfo() override {
    std::cout << "[depth_walk] actor: " << actor_path_ << "\n"
              << "[depth_walk] encoder: " << depth_path_ << "\n"
              << "[depth_walk] proprio=" << kProprioDim << ", latent=" << latent_dim_
              << ", actions=" << kActionDim << ", ready=" << std::boolalpha << ready_
              << std::noboolalpha << std::endl;
  }

  void OnEnter() override {
    action_ = VecXf::Zero(kActionDim);
    last_action_ = VecXf::Zero(kActionDim);
    for (auto& history : term_history_) {
      history.clear();
    }
    depth_history_.clear();
    history_initialized_ = false;
    last_depth_sequence_ = 0;
    joint_target_print_counter_ = 0;
    last_command_print_time_ = std::chrono::steady_clock::time_point{};
    cmd_vel_input_.setZero();
    run_cnt_ = 0;
  }

  void OnExit() override {}

  bool SetRawDepthFrame(const std::vector<float>& raw, int width, int height, uint64_t sequence) {
    cv::Mat processed = PreprocessDepth(raw, width, height);
    if (processed.empty()) {
      return false;
    }
    if (depth_history_.empty()) {
      for (int i = 0; i < kDepthHistoryLength; ++i) {
        depth_history_.push_back(processed.clone());
      }
    } else {
      depth_history_.pop_front();
      depth_history_.push_back(processed);
    }
    last_depth_sequence_ = sequence;
    return true;
  }

  RobotAction GetRobotAction(const RobotBasicState& robot, const UserCommand& user) override {
    if (!ready_ || depth_history_.size() != kDepthHistoryLength) {
      throw std::runtime_error("depth_walk inference requested before policy/depth is ready");
    }

    const Vec3f raw_target(user.forward_vel_scale, user.side_vel_scale,
                           user.turnning_vel_scale);
    // This policy was not trained with backward motion. Reject negative X
    // regardless of whether the command came from the keyboard or a joystick.
    Vec3f target(std::clamp(raw_target(0), 0.0f, 1.0f),
                 std::clamp(raw_target(1), -0.5f, 0.5f),
                 std::clamp(raw_target(2), -1.0f, 1.0f));
    Vec3f delta = target - cmd_vel_input_;
    for (int i = 0; i < 3; ++i) {
      delta(i) = std::clamp(delta(i), -vel_delta_const_(i), vel_delta_const_(i));
    }
    cmd_vel_input_ += delta;
    cmd_vel_input_(0) = std::max(0.0f, cmd_vel_input_(0));
    const Vec3f command = cmd_vel_input_.cwiseProduct(max_cmd_vel_);

    const auto command_print_time = std::chrono::steady_clock::now();
    if (command_print_time - last_command_print_time_ >= std::chrono::seconds(1)) {
      last_command_print_time_ = command_print_time;
      // `command` is the physical velocity presented to the policy before its
      // observation scale (command_scale_ is currently [1, 1, 1]).  Keep the
      // joystick scales as diagnostics, but label the policy input explicitly
      // so they are not mistaken for m/s or rad/s.
      std::cout << "[depth_policy_cmd] x=" << command(0) << " m/s"
                << " y=" << command(1) << " m/s"
                << " yaw=" << command(2) << " rad/s"
                << " | source_scale=" << raw_target.transpose()
                << " accepted_scale=" << target.transpose()
                << " filtered_scale=" << cmd_vel_input_.transpose() << std::endl;
    }
    const Vec3f gravity = robot.base_rot_mat.transpose() * Vec3f(0.0f, 0.0f, -1.0f);

    VecXf joint_pos(kActionDim);
    VecXf joint_vel(kActionDim);
    for (int i = 0; i < kActionDim; ++i) {
      joint_pos(i) = robot.joint_pos(robot_to_policy_idx_[i]) - default_pos_(i);
      joint_vel(i) = robot.joint_vel(robot_to_policy_idx_[i]) * joint_velocity_scale_;
    }
    std::array<VecXf, 6> terms = {
        robot.base_omega * omega_scale_, gravity, command.cwiseProduct(command_scale_),
        joint_pos, joint_vel, last_action_.cwiseMax(-clip_observations_).cwiseMin(clip_observations_)};
    for (auto& term : terms) {
      term = term.cwiseMax(-clip_observations_).cwiseMin(clip_observations_);
    }
    AppendTerms(terms);
    VecXf proprio = FlattenTerms();

    std::vector<float> depth_input(8 * kEncoderHeight * kEncoderWidth);
    size_t depth_offset = 0;
    for (const int frame_index : kDepthIndices) {
      const cv::Mat& frame = depth_history_[frame_index];
      std::copy(frame.ptr<float>(), frame.ptr<float>() + frame.total(),
                depth_input.begin() + depth_offset);
      depth_offset += frame.total();
    }
    const std::array<int64_t, 4> depth_shape = {1, 8, kEncoderHeight, kEncoderWidth};
    auto depth_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, depth_input.data(), depth_input.size(), depth_shape.data(), depth_shape.size());
    const char* depth_inputs[] = {depth_input_name_.c_str()};
    const char* depth_outputs[] = {depth_output_name_.c_str()};
    auto depth_result = depth_session_.Run(Ort::RunOptions{nullptr}, depth_inputs, &depth_tensor, 1,
                                           depth_outputs, 1);
    const float* latent = depth_result[0].GetTensorData<float>();

    std::vector<float> actor_input(kProprioDim + latent_dim_);
    std::copy(proprio.data(), proprio.data() + proprio.size(), actor_input.begin());
    std::copy(latent, latent + latent_dim_, actor_input.begin() + kProprioDim);
    const std::array<int64_t, 2> actor_shape = {1, static_cast<int64_t>(actor_input.size())};
    auto actor_tensor = Ort::Value::CreateTensor<float>(
        memory_info_, actor_input.data(), actor_input.size(), actor_shape.data(), actor_shape.size());
    const char* actor_inputs[] = {actor_input_name_.c_str()};
    const char* actor_outputs[] = {actor_output_name_.c_str()};
    auto actor_result = actor_session_.Run(Ort::RunOptions{nullptr}, actor_inputs, &actor_tensor, 1,
                                           actor_outputs, 1);
    action_ = Eigen::Map<const VecXf>(actor_result[0].GetTensorData<float>(), kActionDim);
    last_action_ = action_;

    RobotAction result;
    result.goal_joint_pos = VecXf::Zero(kActionDim);
    result.goal_joint_vel = VecXf::Zero(kActionDim);
    result.tau_ff = VecXf::Zero(kActionDim);
    result.kp = VecXf::Zero(kActionDim);
    result.kd = VecXf::Zero(kActionDim);
    for (int output_index = 0; output_index < kActionDim; ++output_index) {
      const int policy_index = policy_to_output_idx_[output_index];
      result.goal_joint_pos(output_index) =
          action_scale_ * action_(policy_index) + default_pos_(policy_index);
      result.kp(output_index) = kp_(policy_index);
      result.kd(output_index) = kd_(policy_index);
    }
    PrintJointTargets(result.goal_joint_pos);
    // result.goal_joint_pos.setZero();

    ++run_cnt_;
    return result;
  }
};
