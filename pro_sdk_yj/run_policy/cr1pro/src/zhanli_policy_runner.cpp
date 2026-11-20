#include "zhanli_policy_runner.h"

#include <algorithm>
#include <sstream>

namespace {
constexpr const char* kInputNames[] = {"obs", "time_step"};
constexpr const char* kOutputNames[] = {
    "actions", "joint_pos", "joint_vel", "body_pos_w",
    "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"};
constexpr const char* kJointOrder =
    "left_hip_pitch_joint,right_hip_pitch_joint,waist_yaw_joint,"
    "left_hip_roll_joint,right_hip_roll_joint,waist_roll_joint,"
    "left_hip_yaw_joint,right_hip_yaw_joint,waist_pitch_joint,"
    "left_knee_pitch_joint,right_knee_pitch_joint,"
    "left_shoulder_pitch_joint,right_shoulder_pitch_joint,"
    "left_ankle_pitch_joint,right_ankle_pitch_joint,"
    "left_shoulder_roll_joint,right_shoulder_roll_joint,"
    "left_ankle_roll_joint,right_ankle_roll_joint,"
    "left_shoulder_yaw_joint,right_shoulder_yaw_joint,"
    "left_elbow_pitch_joint,right_elbow_pitch_joint,"
    "left_wrist_yaw_joint,right_wrist_yaw_joint,"
    "left_wrist_pitch_joint,right_wrist_pitch_joint,"
    "left_wrist_roll_joint,right_wrist_roll_joint";

std::string OptionalMetadata(const Ort::ModelMetadata& metadata, const char* key) {
    Ort::AllocatorWithDefaultOptions allocator;
    auto value = metadata.LookupCustomMetadataMapAllocated(key, allocator);
    return value ? std::string(value.get()) : std::string();
}

std::string RequiredMetadata(const Ort::ModelMetadata& metadata, const char* key) {
    const std::string value = OptionalMetadata(metadata, key);
    if (value.empty())
        throw std::runtime_error(std::string("Missing zhanli policy metadata: ") + key);
    return value;
}

VecXf ParseVector(const Ort::ModelMetadata& metadata, const char* key, bool positive) {
    std::istringstream stream(RequiredMetadata(metadata, key));
    std::string item;
    VecXf value(29);
    int count = 0;
    while (std::getline(stream, item, ',')) {
        if (count == value.size())
            throw std::runtime_error(std::string("Too many zhanli parameters: ") + key);
        std::size_t consumed = 0;
        value(count++) = std::stof(item, &consumed);
        if (consumed != item.size())
            throw std::runtime_error(std::string("Invalid zhanli parameter: ") + key);
    }
    if (count != value.size() || !value.allFinite() ||
        (positive && (value.array() <= 0.f).any()))
        throw std::runtime_error(std::string("Invalid zhanli metadata: ") + key);
    return value;
}

float ParseScalar(const std::string& text, const char* key) {
    std::size_t consumed = 0;
    const float value = std::stof(text, &consumed);
    if (consumed != text.size() || !std::isfinite(value))
        throw std::runtime_error(std::string("Invalid zhanli metadata: ") + key);
    return value;
}
}  // namespace

ZhanliPolicyRunner::ZhanliPolicyRunner(const std::string& policy_name,
                                       RobotName robot_name,
                                       const std::string& onnx_path)
    : PolicyRunnerBase(policy_name, robot_name), onnx_path_(onnx_path) {
    decimation_ = 20;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetInterOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    try {
        LoadModel();
    } catch (const std::exception& error) {
        model_loaded_ = false;
        SetInferenceFault(error.what());
    }
}

void ZhanliPolicyRunner::LoadModel() {
    model_loaded_ = false;
    if (onnx_path_.empty()) {
        const char* configured = std::getenv("DR2_ZHANLI_MODEL_PATH");
        onnx_path_ = configured && *configured
            ? std::string(configured)
            : pro_policy::ResolveModelPath(
                  "recovery_policy/zhanli/model_8000.onnx");
    }

    session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
    pro_policy::ValidateCopiedModel(session_, kObservationSize, true);
    const auto metadata = session_.GetModelMetadata();
    if (RequiredMetadata(metadata, "joint_names") != kJointOrder ||
        RequiredMetadata(metadata, "observation_names") !=
            "command,projected_gravity,base_ang_vel,joint_pos,joint_vel,actions" ||
        RequiredMetadata(metadata, "anchor_body_name") != "base_link")
        throw std::runtime_error("Zhanli policy observation or joint order mismatch");

    std::string fps = OptionalMetadata(metadata, "fps");
    if (fps.empty()) fps = OptionalMetadata(metadata, "control_fps");
    if (ParseScalar(fps, "fps/control_fps") != 50.0f)
        throw std::runtime_error("Zhanli policy must run at 50 Hz");

    defaults_ = ParseVector(metadata, "default_joint_pos", false);
    action_scale_ = ParseVector(metadata, "action_scale", true);
    kp_ = ParseVector(metadata, "joint_stiffness", true);
    kd_ = ParseVector(metadata, "joint_damping", true);
    const std::string clip = OptionalMetadata(metadata, "clip_actions");
    clip_actions_ = clip.empty() ? 10.0f : ParseScalar(clip, "clip_actions");
    if (clip_actions_ <= 0.f)
        throw std::runtime_error("Zhanli policy clip_actions must be positive");
    model_loaded_ = true;
}

std::vector<Ort::Value> ZhanliPolicyRunner::Infer(const VecXf& observation, int frame) {
    pro_policy::ValidateObservation(observation, kObservationSize);
    const int64_t observation_shape[]{1, kObservationSize};
    const int64_t time_shape[]{1, 1};
    float time_step = static_cast<float>(frame);
    std::array<Ort::Value, 2> inputs{
        Ort::Value::CreateTensor<float>(
            memory_info_, const_cast<float*>(observation.data()), kObservationSize,
            observation_shape, 2),
        Ort::Value::CreateTensor<float>(
            memory_info_, &time_step, 1, time_shape, 2)};
    auto outputs = session_.Run(
        Ort::RunOptions{nullptr}, kInputNames, inputs.data(), inputs.size(),
        kOutputNames, 7);
    pro_policy::ValidateMimicOutputs(outputs);
    return outputs;
}

void ZhanliPolicyRunner::CacheReference(
        const std::vector<Ort::Value>& outputs) {
    reference_pos_ =
        Eigen::Map<const VecXf>(outputs[1].GetTensorData<float>(), kActionSize);
    reference_vel_ =
        Eigen::Map<const VecXf>(outputs[2].GetTensorData<float>(), kActionSize);
}

void ZhanliPolicyRunner::OnEnter() {
    ResetInferenceFault();
    if (!model_loaded_) LoadModel();
    run_cnt_ = 0;
    actor_action_.setZero();
    last_action_.setZero();

    const VecXf zero = VecXf::Zero(kObservationSize);
    CacheReference(Infer(zero, 0));
}

void ZhanliPolicyRunner::OnExit() {
    run_cnt_ = 0;
    actor_action_.setZero();
    last_action_.setZero();
    reference_pos_.setZero();
    reference_vel_.setZero();
}

RobotAction ZhanliPolicyRunner::ComputeRobotAction(
        const RobotBasicState& state, const UserCommand&) {
    VecXf q(kActionSize), dq(kActionSize), observation(kObservationSize);
    for (int i = 0; i < kActionSize; ++i) {
        q(i) = state.joint_pos(kPolicyToRobot[i]);
        dq(i) = state.joint_vel(kPolicyToRobot[i]);
    }
    const Vec3f gravity =
        state.base_rot_mat.transpose() * Vec3f(0.f, 0.f, -1.f);
    observation << reference_pos_, reference_vel_, gravity, state.base_omega,
        q - defaults_, dq, last_action_;

    const int inference_frame = run_cnt_;
    const auto outputs = Infer(observation, inference_frame);
    actor_action_ =
        Eigen::Map<const VecXf>(outputs[0].GetTensorData<float>(), kActionSize);
    last_action_ = actor_action_.cwiseMax(-clip_actions_).cwiseMin(clip_actions_);
    CacheReference(outputs);
    if (inference_frame == kLastFrame) reference_vel_.setZero();
    if (run_cnt_ < kLastFrame) ++run_cnt_;

    RobotAction action;
    action.goal_joint_pos = VecXf::Zero(kActionSize);
    action.goal_joint_vel = VecXf::Zero(kActionSize);
    action.kp = VecXf::Zero(kActionSize);
    action.kd = VecXf::Zero(kActionSize);
    action.tau_ff = VecXf::Zero(kActionSize);
    for (int i = 0; i < kActionSize; ++i) {
        const int robot_joint = kPolicyToRobot[i];
        action.goal_joint_pos(robot_joint) =
            defaults_(i) + action_scale_(i) * last_action_(i);
        action.kp(robot_joint) = kp_(i);
        action.kd(robot_joint) = kd_(i);
    }
    return action;
}

void ZhanliPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_
              << ": zhanli, 151 observations / 29 actions, 50 Hz, "
              << kLastFrame + 1 << " frames, metadata PD, model " << onnx_path_
              << std::endl;
}
