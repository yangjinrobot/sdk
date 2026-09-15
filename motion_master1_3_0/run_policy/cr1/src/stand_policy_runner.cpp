#include "stand_policy_runner.h"

#include <fstream>
#include <stdexcept>
#include <unordered_map>

namespace {
std::vector<int> MakePermutation(const std::vector<std::string>& from,
                                 const std::vector<std::string>& to) {
    std::unordered_map<std::string, int> index;
    for (int i = 0; i < static_cast<int>(from.size()); ++i) index[from[i]] = i;
    std::vector<int> result;
    result.reserve(to.size());
    for (const auto& name : to) {
        const auto it = index.find(name);
        if (it == index.end()) throw std::runtime_error("Missing policy joint: " + name);
        result.push_back(it->second);
    }
    return result;
}
}  // namespace

StandPolicyRunner::StandPolicyRunner(const std::string& policy_name, RobotName robot_name,
                                     const std::string& onnx_path)
    : PolicyRunnerBase(policy_name, robot_name), env_(ORT_LOGGING_LEVEL_ERROR, policy_name.c_str()),
      onnx_path_(onnx_path) {
    if (robot_name != RobotName::CR1STANDARD) {
        throw std::runtime_error("stand_4.onnx supports CR1STANDARD (21 DOF) only");
    }

    const std::vector<std::string> robot_order = {
        "waist_z_joint",
        "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint", "left_elbow_joint",
        "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint", "right_elbow_joint",
        "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint", "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
        "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint", "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint"};
    const std::vector<std::string> policy_order = {
        "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint", "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
        "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint", "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint",
        "waist_z_joint",
        "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint", "left_elbow_joint",
        "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint", "right_elbow_joint"};
    robot2policy_idx_ = MakePermutation(robot_order, policy_order);
    policy2robot_idx_ = MakePermutation(policy_order, robot_order);

    kp_.resize(kActionDim);
    kd_.resize(kActionDim);
    kp_ << 300.f, 300.f, 300.f, 300.f, 80.f, 80.f,
           300.f, 300.f, 300.f, 300.f, 80.f, 80.f,
           200.f,
           100.f, 100.f, 100.f, 100.f,
           100.f, 100.f, 100.f, 100.f;
    kp_ = kp_ * 1.4;
    kd_ << 10.f, 10.f, 10.f, 10.f, 3.f, 3.f,
           10.f, 10.f, 10.f, 10.f, 3.f, 3.f,
           10.f,
           5.f, 5.f, 5.f, 5.f,
           5.f, 5.f, 5.f, 5.f;

    decimation_ = 20;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::ifstream policy_file(onnx_path_);
    if (!policy_file.good()) throw std::runtime_error("Stand ONNX not found: " + onnx_path_);
    session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
    std::cout << "loaded " << policy_name_ << " ONNX model from: "
              << onnx_path_ << std::endl;
    OnEnter();
}

void StandPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << ": 420-D HoST stand policy, 21 actions" << std::endl;
}

void StandPolicyRunner::OnEnter() {
    q_policy_.setZero(kActionDim);
    dq_policy_.setZero(kActionDim);
    action_.setZero(kActionDim);
    last_action_.setZero(kActionDim);
    observation_history_.setZero(kObservationDim * kHistoryFrames);
    observation_.setZero(kObservationDim);
    run_cnt_ = 0;
}

RobotAction StandPolicyRunner::GetRobotAction(const RobotBasicState& ro, const UserCommand&) {
    for (int i = 0; i < kActionDim; ++i) {
        q_policy_(i) = ro.joint_pos(robot2policy_idx_[i]);
        dq_policy_(i) = ro.joint_vel(robot2policy_idx_[i]);
    }

    observation_.setZero();
    if (run_cnt_ > kUnactuatedSteps) {
        const Vec3f projected_gravity = ro.base_rot_mat.transpose() * Vec3f(0.f, 0.f, -1.f);
        observation_ << 0.25f * ro.base_omega, projected_gravity, q_policy_,
                        0.05f * dq_policy_, last_action_, kActionScale;
    }
    observation_ = observation_.cwiseMax(-100.f).cwiseMin(100.f);
    observation_history_.head((kHistoryFrames - 1) * kObservationDim) =
        observation_history_.tail((kHistoryFrames - 1) * kObservationDim).eval();
    observation_history_.tail(kObservationDim) = observation_;

    if (run_cnt_ <= kUnactuatedSteps) {
        action_.setZero();
    } else {
        const std::vector<int64_t> shape = {1, kObservationDim * kHistoryFrames};
        auto input = Ort::Value::CreateTensor<float>(memory_info_, observation_history_.data(),
            observation_history_.size(), shape.data(), shape.size());
        ort_inputs_.clear();
        ort_inputs_.push_back(std::move(input));
        auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, ort_inputs_.data(),
                                    ort_inputs_.size(), output_names_, 1);
        action_ = Eigen::Map<VecXf>(outputs[0].GetTensorMutableData<float>(), kActionDim);
        action_ = action_.cwiseMax(-100.f).cwiseMin(100.f);
    }
    last_action_ = action_;

    RobotAction result;
    result.goal_joint_pos = VecXf::Zero(kActionDim);
    result.goal_joint_vel = VecXf::Zero(kActionDim);
    result.tau_ff = VecXf::Zero(kActionDim);
    result.kp = VecXf::Zero(kActionDim);
    result.kd = VecXf::Zero(kActionDim);
    for (int robot_idx = 0; robot_idx < kActionDim; ++robot_idx) {
        const int policy_idx = policy2robot_idx_[robot_idx];
        result.goal_joint_pos(robot_idx) = ro.joint_pos(robot_idx) + kActionScale * action_(policy_idx);
        result.kp(robot_idx) = kp_(policy_idx);
        result.kd(robot_idx) = kd_(policy_idx);
    }
    ++run_cnt_;
    return result;
}
