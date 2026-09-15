#include "stand_shooting_policy_runner.h"

#include <stdexcept>

StandShootingPolicyRunner::StandShootingPolicyRunner(const std::string& policy_name, RobotName robot_name,
    const std::string& onnx_path) : PolicyRunnerBase(policy_name, robot_name),
    env_(ORT_LOGGING_LEVEL_ERROR, policy_name.c_str()), onnx_path_(onnx_path) {
    if (robot_name != RobotName::CR1STANDARD)
        throw std::runtime_error("zhanli_shooting.onnx supports CR1STANDARD (21 DOF) only");
    default_q_.resize(kDof);
    kp_.resize(kDof);
    kd_.resize(kDof);
    // Robot DOF order: waist, left arm, right arm, left leg, right leg.
    // Values are aligned with B3STD_config(1).py / init_state.default_joint_angles.
    default_q_ << 0.0, 0.0, 0.15, 0.0, 1.2, 0.0, -0.15, 0.0, 1.2,
        -0.2, 0.0, 0.0, 0.4, -0.2, 0.0, -0.2, 0.0, 0.0, 0.4, -0.2, 0.0;
    kp_ << 200.f, 100.f, 100.f, 100.f, 100.f, 100.f, 100.f, 100.f, 100.f,
        300.f, 300.f, 200.f, 300.f, 50.f, 30.f, 300.f, 300.f, 200.f, 300.f, 50.f, 30.f;
    kp_ = kp_ * 0.8;
    kd_ << 5.f, 3.f, 3.f, 3.f, 3.f, 3.f, 3.f, 3.f, 3.f,
        10.f, 10.f, 5.f, 10.f, 2.f, 1.f, 10.f, 10.f, 5.f, 10.f, 2.f, 1.f;
    decimation_ = 10;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::ifstream policy_file(onnx_path_);
    if (!policy_file.good()) throw std::runtime_error("Stand-shooting ONNX not found: " + onnx_path_);
    session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
    std::cout << "loaded " << policy_name_ << " ONNX model from: "
              << onnx_path_ << std::endl;
    OnEnter();
}

void StandShootingPolicyRunner::DisplayPolicyInfo() { std::cout << policy_name_ << ": 396-D stand-shooting policy, 13 actions" << std::endl; }
void StandShootingPolicyRunner::OnEnter() {
    action_.setZero(kActionDim); last_action_.setZero(kActionDim); observation_.setZero(kObsDim);
    observation_history_.setZero(kObsDim * kHistoryFrames); run_cnt_ = 0;
}

RobotAction StandShootingPolicyRunner::GetRobotAction(const RobotBasicState& ro, const UserCommand& uc) {
    VecXf q(kActionDim), dq(kActionDim), policy_default(kActionDim);
    for (int i = 0; i < kActionDim; ++i) {
        q(i) = ro.joint_pos(policy_to_robot_[i]); dq(i) = ro.joint_vel(policy_to_robot_[i]);
        policy_default(i) = default_q_(policy_to_robot_[i]);
    }
    const Vec3f command(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);
    const Vec3f gravity = ro.base_rot_mat.transpose() * Vec3f(0.f, 0.f, -1.f);
    const float phase = (std::hypot(command.x(), command.y()) > 0.1f) ? static_cast<float>(run_cnt_) * 0.01f / 0.8f : 0.f;
    observation_ << ro.base_omega, gravity, 2.f * command.x(), 2.f * command.y(), command.z(),
        q - policy_default, 0.05f * dq, last_action_, std::sin(2.f * static_cast<float>(M_PI) * phase),
        std::cos(2.f * static_cast<float>(M_PI) * phase);
    observation_ = observation_.cwiseMax(-100.f).cwiseMin(100.f);
    observation_history_.head((kHistoryFrames - 1) * kObsDim) = observation_history_.tail((kHistoryFrames - 1) * kObsDim).eval();
    observation_history_.tail(kObsDim) = observation_;
    const std::vector<int64_t> shape = {1, kObsDim * kHistoryFrames};
    auto input = Ort::Value::CreateTensor<float>(memory_info_, observation_history_.data(), observation_history_.size(), shape.data(), shape.size());
    ort_inputs_.clear(); ort_inputs_.push_back(std::move(input));
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, ort_inputs_.data(), ort_inputs_.size(), output_names_, 1);
    action_ = Eigen::Map<VecXf>(outputs[0].GetTensorMutableData<float>(), kActionDim).cwiseMax(-100.f).cwiseMin(100.f);
    last_action_ = action_;
    RobotAction result;
    result.goal_joint_pos = default_q_; result.goal_joint_vel = VecXf::Zero(kDof); result.tau_ff = VecXf::Zero(kDof); result.kp = kp_; result.kd = kd_;
    for (int i = 0; i < kActionDim; ++i) result.goal_joint_pos(policy_to_robot_[i]) = policy_default(i) + kActionScale * action_(i);
    ++run_cnt_;
    return result;
}
