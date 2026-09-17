#include "shoot_up_policy_runner.hpp"
#include "shoot_reference_loader.hpp"

#include <array>
#include <stdexcept>
namespace {
constexpr int kJointCount = 21;
constexpr int kObservationSize = 114;
constexpr int N = kJointCount;
constexpr int O = kObservationSize;
void SetDeploymentParameters(VecXf& q, VecXf& kp, VecXf& kd, VecXf& scale) {
    q.resize(kJointCount); kp.resize(kJointCount); kd.resize(kJointCount); scale.resize(kJointCount);
    q << -.2f, -.2f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, .15f, -.15f,
        .4f, .4f, 0.f, 0.f, -.2f, -.2f, 1.2f, 1.2f, 0.f, 0.f;
    kp << 100.f, 100.f, 100.f, 100.f, 100.f, 50.f, 50.f, 80.f, 80.f, 50.f, 50.f,
        100.f, 100.f, 50.f, 50.f, 70.f, 70.f, 50.f, 50.f, 30.f, 30.f;
    kd << 3.f, 3.f, 3.f, 3.f, 3.f, 5.f, 5.f, 3.f, 3.f, 5.f, 5.f,
        3.f, 3.f, 5.f, 5.f, 2.f, 2.f, 5.f, 5.f, 1.f, 1.f;
    scale << .908f, .908f, .343f, .908f, .908f, .685f, .685f, .428f, .428f, .685f, .685f,
        .908f, .908f, .685f, .685f, .489f, .489f, .685f, .685f, .417f, .417f;
}
}  // namespace
ShootUpPolicyRunner::ShootUpPolicyRunner(RobotName robot_name)
    : PolicyRunnerBase("shoot_up", robot_name) {
    if (robot_name != RobotName::CR1STANDARD) throw std::runtime_error("shoot_up requires CR1STANDARD");
    SetDeploymentParameters(default_q_, kp_, kd_, action_scale_);
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, "/opt/robot/share/motion_master/policy/mimic/model_28000.onnx", session_options_);
    LoadReference();
    if (FrameCount() != kFrameCount) throw std::runtime_error("shoot_up.npz must contain 350 frames");
    decimation_ = 20;
    OnEnter();
}
void ShootUpPolicyRunner::LoadReference() {
    for (const auto& frame : LoadShootReferenceNpz(
             "/opt/robot/share/motion_master/policy/mimic/shoot_up.npz")) {
        reference_.push_back(
            {frame.joint_position, frame.joint_velocity, frame.anchor_rotation});
    }
}

VecXf ShootUpPolicyRunner::ToRobotOrder(const VecXf& policy_order) const {
    VecXf robot_order(N);
    for (int index = 0; index < N; ++index) {
        robot_order(policy_to_robot_[index]) = policy_order(index);
    }
    return robot_order;
}

void ShootUpPolicyRunner::DisplayPolicyInfo() {
    std::cout << "shoot_up: 114 observations, 350 frames at 50 Hz" << std::endl;
}

void ShootUpPolicyRunner::OnEnter() {
    frame_ = 0;
    final_frame_was_run_ = false;
    run_cnt_ = 0;
    previous_action_ = VecXf::Zero(N);
    observation_.setZero();
    heading_alignment_.setIdentity();
}

void ShootUpPolicyRunner::AlignHeading(const Mat3f& actual) {
    const auto& reference = reference_.front().anchor_rotation;
    const float yaw = std::atan2(actual(1, 0), actual(0, 0)) -
        std::atan2(reference(1, 0), reference(0, 0));
    heading_alignment_ = Eigen::AngleAxisf(yaw, Vec3f::UnitZ()).toRotationMatrix();
}

bool ShootUpPolicyRunner::HoldingFinalFrame() const {
    return !reference_.empty() && frame_ == static_cast<int>(reference_.size()) - 1;
}

int ShootUpPolicyRunner::Frame() const { return frame_; }
int ShootUpPolicyRunner::FrameCount() const { return static_cast<int>(reference_.size()); }

RobotAction ShootUpPolicyRunner::GetRobotAction(const RobotBasicState& robot, const UserCommand&) {
    const auto& reference = reference_[frame_];
    observation_.segment(0, N) = reference.joint_position;
    if (final_frame_was_run_) {
        observation_.segment(N, N).setZero();
    } else {
        observation_.segment(N, N) = reference.joint_velocity;
    }
    const auto relative_rotation =
        robot.base_rot_mat.transpose() * heading_alignment_ * reference.anchor_rotation;
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
    std::array<int64_t, 2> observation_shape{1, O}, time_shape{1, 1};
    std::array<Ort::Value, 2> inputs{
        Ort::Value::CreateTensor<float>(memory_info_, observation_.data(), O,
                                        observation_shape.data(), 2),
        Ort::Value::CreateTensor<float>(memory_info_, &time_step, 1,
                                        time_shape.data(), 2)};
    const char* output_names[]{"actions"};
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, inputs.data(),
                                2, output_names, 1);
    const auto action = Eigen::Map<const VecXf>(outputs[0].GetTensorData<float>(), N);
    RobotAction result;
    result.goal_joint_pos = ToRobotOrder(default_q_ + action_scale_.cwiseProduct(action));
    result.goal_joint_vel = VecXf::Zero(N);
    result.tau_ff = VecXf::Zero(N);
    result.kp = ToRobotOrder(kp_);
    result.kd = ToRobotOrder(kd_);
    previous_action_ = action;
    if (frame_ + 1 < static_cast<int>(reference_.size())) {
        ++frame_;
    } else {
        final_frame_was_run_ = true;
    }
    ++run_cnt_;
    return result;
}
