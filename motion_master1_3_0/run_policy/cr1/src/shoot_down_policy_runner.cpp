#include "shoot_down_policy_runner.hpp"
#include "shoot_reference_loader.hpp"

#include <array>
#include <stdexcept>

namespace {
constexpr int kJointCount = 21;
constexpr int kObservationSize = 114;
constexpr int N = kJointCount;
constexpr int O = kObservationSize;

void SetDeploymentParameters(VecXf& default_q, VecXf& kp, VecXf& kd, VecXf& action_scale) {
    default_q.resize(kJointCount);
    kp.resize(kJointCount);
    kd.resize(kJointCount);
    action_scale.resize(kJointCount);
    default_q << -0.2f, -0.2f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.15f, -0.15f,
        0.4f, 0.4f, 0.f, 0.f, -0.2f, -0.2f, 1.2f, 1.2f, 0.f, 0.f;
    kp << 100.f, 100.f, 100.f, 100.f, 100.f, 50.f, 50.f, 80.f, 80.f, 50.f, 50.f,
        100.f, 100.f, 50.f, 50.f, 70.f, 70.f, 50.f, 50.f, 30.f, 30.f;
    kd << 3.f, 3.f, 3.f, 3.f, 3.f, 5.f, 5.f, 3.f, 3.f, 5.f, 5.f,
        3.f, 3.f, 5.f, 5.f, 2.f, 2.f, 5.f, 5.f, 1.f, 1.f;
    action_scale << 0.908f, 0.908f, 0.343f, 0.908f, 0.908f, 0.685f, 0.685f,
        0.428f, 0.428f, 0.685f, 0.685f, 0.908f, 0.908f, 0.685f, 0.685f,
        0.489f, 0.489f, 0.685f, 0.685f, 0.417f, 0.417f;
}
}  // namespace
ShootDownPolicyRunner::ShootDownPolicyRunner(RobotName robot_name)
    : PolicyRunnerBase("shoot_down", robot_name) {
    if (robot_name != RobotName::CR1STANDARD)
        throw std::runtime_error("shoot_down requires CR1STANDARD");
    SetDeploymentParameters(default_q_, kp_, kd_, action_scale_);
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, "/opt/robot/share/motion_master/policy/mimic/model_43000.onnx", session_options_);
    LoadReference();
    if (FrameCount() != kFrameCount) throw std::runtime_error("shoot_down.npz must contain 267 frames");
    decimation_ = 20;
    OnEnter();
}
void ShootDownPolicyRunner::LoadReference() {
    for (const auto& frame : LoadShootReferenceNpz("/opt/robot/share/motion_master/policy/mimic/shoot_down.npz"))
        reference_.push_back({frame.joint_position, frame.joint_velocity, frame.anchor_rotation});
}
VecXf ShootDownPolicyRunner::ToRobotOrder(const VecXf& policy_order) const {
    VecXf robot_order(kJointCount);
    for (int i = 0; i < kJointCount; ++i) robot_order(policy_to_robot_[i]) = policy_order(i);
    return robot_order;
}
void ShootDownPolicyRunner::DisplayPolicyInfo() { std::cout << "shoot_down: 114 observations, 267 frames at 50 Hz" << std::endl; }
void ShootDownPolicyRunner::OnEnter() {
    frame_ = 0;
    final_frame_was_run_ = false;
    run_cnt_ = 0;
    previous_action_ = VecXf::Zero(kJointCount);
    observation_.setZero();
    heading_alignment_.setIdentity();
}
void ShootDownPolicyRunner::AlignHeading(const Mat3f& actual) { const auto& ref = reference_.front().anchor_rotation; const float yaw = std::atan2(actual(1, 0), actual(0, 0)) - std::atan2(ref(1, 0), ref(0, 0)); heading_alignment_ = Eigen::AngleAxisf(yaw, Vec3f::UnitZ()).toRotationMatrix(); }
bool ShootDownPolicyRunner::HoldingFinalFrame() const {
    return !reference_.empty() && frame_ == static_cast<int>(reference_.size()) - 1;
}
int ShootDownPolicyRunner::Frame() const { return frame_; }
int ShootDownPolicyRunner::FrameCount() const { return static_cast<int>(reference_.size()); }
RobotAction ShootDownPolicyRunner::GetRobotAction(const RobotBasicState& robot, const UserCommand&) {
    const auto& reference = reference_[frame_];
    observation_.segment(0, N) = reference.joint_position;
    // commands.py keeps the final position but zeros its reference velocity
    // on every update after the final sample has first been consumed.
    if (final_frame_was_run_) {
        observation_.segment(N, N).setZero();
    } else {
        observation_.segment(N, N) = reference.joint_velocity;
    }
    const Mat3f relative_rotation = robot.base_rot_mat.transpose() * heading_alignment_ * reference.anchor_rotation;
    for (int row = 0; row < 3; ++row)
        for (int column = 0; column < 2; ++column)
            observation_(42 + row * 2 + column) = relative_rotation(row, column);
    observation_.segment<3>(48) = robot.base_omega;
    for (int index = 0; index < N; ++index) {
        observation_(51 + index) = robot.joint_pos(policy_to_robot_[index]) - default_q_(index);
        observation_(72 + index) = robot.joint_vel(policy_to_robot_[index]);
    }
    observation_.tail(N) = previous_action_;
    float time_step = static_cast<float>(frame_);
    const std::array<int64_t, 2> observation_shape{1, O}, time_shape{1, 1};
    std::array<Ort::Value, 2> inputs{
        Ort::Value::CreateTensor<float>(memory_info_, observation_.data(), O, observation_shape.data(), 2),
        Ort::Value::CreateTensor<float>(memory_info_, &time_step, 1, time_shape.data(), 2)};
    const char* output_names[]{"actions"};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, inputs.data(), 2, output_names, 1);
    const VecXf action = Eigen::Map<const VecXf>(outputs[0].GetTensorData<float>(), N);
    RobotAction result;
    result.goal_joint_pos = ToRobotOrder(default_q_ + action_scale_.cwiseProduct(action));
    result.goal_joint_vel = VecXf::Zero(N);
    result.tau_ff = VecXf::Zero(N);
    result.kp = ToRobotOrder(kp_);
    result.kd = ToRobotOrder(kd_);
    previous_action_ = action;
    // Training clamps the motion command to its final sample after playback;
    // keep the actor closed-loop instead of reusing this action indefinitely.
    if (frame_ + 1 < static_cast<int>(reference_.size())) {
        ++frame_;
    } else {
        final_frame_was_run_ = true;
    }
    ++run_cnt_;
    return result;
}
