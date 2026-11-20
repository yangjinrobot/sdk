#include "sit2stand_mimic_policy_runner.hpp"

#include "json.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {
std::vector<std::string> Split(const std::string& text) {
    std::vector<std::string> result;
    std::stringstream stream(text);
    std::string value;
    while (std::getline(stream, value, ',')) result.push_back(value);
    return result;
}

VecXf TrainingVector(const nlohmann::json& training, const char* key) {
    const auto values = training.at(key).get<std::vector<float>>();
    if (values.size() != Sit2StandMimicPolicyRunner::kJoints) throw std::runtime_error(std::string("Invalid sit2stand training ") + key);
    VecXf result = Eigen::Map<const VecXf>(values.data(), Sit2StandMimicPolicyRunner::kJoints);
    if (!result.allFinite()) throw std::runtime_error("Non-finite sit2stand training parameters");
    return result;
}
}  // namespace

std::string Sit2StandMimicPolicyRunner::Metadata(const char* key) const {
    Ort::AllocatorWithDefaultOptions allocator;
    auto value = session_.GetModelMetadata().LookupCustomMetadataMapAllocated(key, allocator);
    if (!value) throw std::runtime_error(std::string("Missing mimic metadata: ") + key);
    return value.get();
}

VecXf Sit2StandMimicPolicyRunner::MetadataVector(const char* key) const {
    const auto values = Split(Metadata(key));
    if (values.size() != kJoints) throw std::runtime_error(std::string("Invalid mimic metadata: ") + key);
    VecXf result(kJoints);
    for (int i = 0; i < kJoints; ++i) {
        size_t used = 0;
        result(i) = std::stof(values[i], &used);
        if (used != values[i].size()) throw std::runtime_error("Invalid numeric mimic metadata");
    }
    if (!result.allFinite()) throw std::runtime_error("Non-finite mimic metadata");
    return result;
}

void Sit2StandMimicPolicyRunner::ValidateTensor(bool input, const char* name, const std::vector<int64_t>& shape) const {
    Ort::AllocatorWithDefaultOptions allocator;
    const size_t count = input ? session_.GetInputCount() : session_.GetOutputCount();
    for (size_t i = 0; i < count; ++i) {
        auto actual_name = input ? session_.GetInputNameAllocated(i, allocator) : session_.GetOutputNameAllocated(i, allocator);
        if (std::string(actual_name.get()) != name) continue;
        auto info = input ? session_.GetInputTypeInfo(i) : session_.GetOutputTypeInfo(i);
        auto tensor = info.GetTensorTypeAndShapeInfo();
        if (tensor.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT || tensor.GetShape() != shape)
            throw std::runtime_error(std::string("Invalid mimic tensor: ") + name);
        return;
    }
    throw std::runtime_error(std::string("Missing mimic tensor: ") + name);
}

std::vector<Ort::Value> Sit2StandMimicPolicyRunner::Infer(const VecXf& observation, int frame, const char* const* outputs, size_t count) {
    float time_step = static_cast<float>(frame);
    const std::array<int64_t, 2> obs_shape{1, observation.size()}, time_shape{1, 1};
    std::array<Ort::Value, 2> inputs{
        Ort::Value::CreateTensor<float>(memory_, const_cast<float*>(observation.data()), observation.size(), obs_shape.data(), 2),
        Ort::Value::CreateTensor<float>(memory_, &time_step, 1, time_shape.data(), 2)};
    return session_.Run(Ort::RunOptions{nullptr}, input_names_, inputs.data(), 2, outputs, count);
}

VecXf Sit2StandMimicPolicyRunner::ToRobot(const VecXf& policy) const {
    VecXf robot(kJoints);
    for (int i = 0; i < kJoints; ++i) robot(policy_to_robot_[i]) = policy(i);
    return robot;
}

Sit2StandMimicPolicyRunner::Sit2StandMimicPolicyRunner(RobotName robot_name,
                                                       const std::filesystem::path& directory)
    : PolicyRunnerBase("sit2stand_mimic", robot_name), observation_(VecXf::Zero(kObs)) {
    if (robot_name != RobotName::CR1STANDARD)
        throw std::invalid_argument("BeyondMimic requires CR1STANDARD / 21 DOF");
    std::ifstream file(directory / "motion.json");
    if (!file) throw std::runtime_error("Cannot open mimic motion.json");
    nlohmann::json config;
    file >> config;
    const int frames = config.at("num_frames").get<int>();
    if (config.at("num_joints").get<int>() != kJoints)
        throw std::runtime_error("BeyondMimic manifest must specify 21 joints");
    if (config.at("fps").get<double>() != 50.0 || frames < 1 || frames > 100000)
        throw std::runtime_error("Mimic reference must be a finite 50 Hz clip");
    const auto path = std::filesystem::canonical(directory / config.at("policy_file").get<std::string>());
    policy_path_ = path;
    options_.SetIntraOpNumThreads(1);
    options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, path.c_str(), options_);
    if (session_.GetInputCount() != 2) throw std::runtime_error("Mimic requires obs and time_step inputs");
    ValidateTensor(true, "obs", {1, observation_.size()});
    ValidateTensor(true, "time_step", {1, 1});
    ValidateTensor(false, "actions", {1, kJoints});
    ValidateTensor(false, "joint_pos", {1, kJoints});
    ValidateTensor(false, "joint_vel", {1, kJoints});
    constexpr const char* observation_names =
        "command,motion_anchor_ori_b,base_ang_vel,joint_pos,joint_vel,actions";
    if (Metadata("observation_names") != observation_names ||
        Metadata("anchor_body_name") != "base_link" || Metadata("command_names") != "motion")
        throw std::runtime_error("Unsupported mimic observation/anchor configuration");
    const auto bodies = Split(Metadata("body_names"));
    const auto anchor = std::find(bodies.begin(), bodies.end(), "base_link");
    if (anchor == bodies.end()) throw std::runtime_error("Mimic reference has no base_link");
    const int anchor_index = static_cast<int>(anchor - bodies.begin());
    ValidateTensor(false, "body_quat_w", {1, static_cast<int64_t>(bodies.size()), 4});
    // Robot order: waist, left arm, right arm, left leg, right leg.
    // Training pitch/roll/yaw names correspond to deployment y/x/z names.
    const std::vector<std::string> sdk_names{
        "waist_yaw_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_pitch_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_pitch_joint",
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"};
    const auto names = Split(Metadata("joint_names"));
    if (names.size() != kJoints) throw std::runtime_error("BeyondMimic requires 21 joints");
    std::vector<bool> seen(kJoints, false);
    for (const auto& name : names) {
        auto it = std::find(sdk_names.begin(), sdk_names.end(), name);
        if (it == sdk_names.end()) throw std::runtime_error("Unknown mimic joint: " + name);
        const int index = static_cast<int>(it - sdk_names.begin());
        if (seen[index]) throw std::runtime_error("Duplicate mimic joint: " + name);
        seen[index] = true;
        policy_to_robot_.push_back(index);
    }
    default_ = MetadataVector("default_joint_pos");
    kp_ = MetadataVector("joint_stiffness");
    kd_ = MetadataVector("joint_damping");
    scale_ = MetadataVector("action_scale");
    if ((kp_.array() <= 0).any() || (kd_.array() < 0).any() || (scale_.array() <= 0).any())
        throw std::runtime_error("Invalid mimic gains or action scale");
    // Verified against the gun_mimic and stand2sit_policy training YAML:
    // scale = 0.25 * effort_limit_sim / stiffness for this B3 export.
    // ONNX metadata rounds scales to 3 decimals; retain the exact training values.
    for (int i = 0; i < kJoints; ++i) {
        const auto& name = names[i];
        const bool high_effort = name.find("hip_pitch") != std::string::npos ||
                                 name.find("hip_roll") != std::string::npos ||
                                 name.find("knee_pitch") != std::string::npos;
        const float effort = high_effort ? 363.0f : 137.0f;
        const float exact_scale = 0.25f * effort / kp_(i);
        if (std::abs(scale_(i) - exact_scale) > 0.00051f)
            throw std::runtime_error("BeyondMimic metadata action_scale disagrees with B3 training effort/KP");
        scale_(i) = exact_scale;
    }
    // Cache reference-only graph outputs before starting control. Frame t's reference
    // must be available BEFORE action inference at t (not one frame behind).
    const char* ref_outputs[] = {"joint_pos", "joint_vel", "body_quat_w"};
    references_.reserve(frames);
    for (int i = 0; i < frames; ++i) {
        auto output = Infer(observation_, i, ref_outputs, 3);
        Reference ref;
        ref.position = Eigen::Map<const VecXf>(output[0].GetTensorData<float>(), kJoints);
        ref.velocity = Eigen::Map<const VecXf>(output[1].GetTensorData<float>(), kJoints);
        const float* q = output[2].GetTensorData<float>() + 4 * anchor_index;
        Eigen::Quaternionf quat(q[0], q[1], q[2], q[3]); // ONNX uses w,x,y,z.
        if (!ref.position.allFinite() || !ref.velocity.allFinite() || !quat.coeffs().allFinite() || quat.norm() < 1e-6f)
            throw std::runtime_error("Invalid embedded mimic reference");
        ref.anchor_rotation = quat.normalized().toRotationMatrix();
        references_.push_back(std::move(ref));
    }
    decimation_ = 20; // Main FSM runs at 1000 Hz; policy and reference run at 50 Hz.
    OnEnter();
    ValidateTraining(directory);
}

void Sit2StandMimicPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << " policy: " << policy_path_.string() << "\n"
              << "BeyondMimic B3: " << observation_.size() << " observations, 21 actions, " << references_.size()
              << " frames at 50 Hz (" << references_.size() * 0.02 << " s)\n"
              << "Mimic kp (SDK order): " << JointKp().transpose() << "\n"
              << "Mimic kd (SDK order): " << JointKd().transpose() << "\n";
}

void Sit2StandMimicPolicyRunner::OnEnter() {
    frame_ = 0;
    holding_last_frame_ = false;
    run_cnt_ = 0;
    previous_action_ = VecXf::Zero(kJoints);
    observation_.setZero();
    heading_alignment_.setIdentity();
}

void Sit2StandMimicPolicyRunner::AlignHeading(const Mat3f& actual) {
    const auto& reference = references_.front().anchor_rotation;
    const float yaw = std::atan2(actual(1, 0), actual(0, 0)) - std::atan2(reference(1, 0), reference(0, 0));
    heading_alignment_ = Eigen::AngleAxisf(yaw, Vec3f::UnitZ()).toRotationMatrix();
}

RobotAction Sit2StandMimicPolicyRunner::GetRobotAction(const RobotBasicState& robot, const UserCommand&) {
    return ComputeAction(robot, true);
}

RobotAction Sit2StandMimicPolicyRunner::GetPreparationAction(const RobotBasicState& robot) {
    if (frame_ != 0) throw std::logic_error("Mimic preparation requires frame zero");
    return ComputeAction(robot, false);
}

RobotAction Sit2StandMimicPolicyRunner::ComputeAction(const RobotBasicState& robot, bool advance) {
    if (robot.joint_pos.size() != kJoints || robot.joint_vel.size() != kJoints ||
        !robot.base_rot_mat.allFinite() || !robot.base_omega.allFinite() ||
        !robot.joint_pos.allFinite() || !robot.joint_vel.allFinite())
        throw std::runtime_error("Invalid mimic robot observation");
    const auto& reference = references_[frame_];
    observation_.segment(0, kJoints) = reference.position;
    observation_.segment(kJoints, kJoints) = reference.velocity;
    int offset = 2 * kJoints;
    const Mat3f relative = robot.base_rot_mat.transpose() * heading_alignment_ * reference.anchor_rotation;
    // NumPy [:, :2].reshape(6) is row-major, unlike Eigen's default storage.
    for (int row = 0; row < 3; ++row)
        for (int col = 0; col < 2; ++col) observation_(offset + 2 * row + col) = relative(row, col);
    offset += 6;
    observation_.segment<3>(offset) = robot.base_omega;
    offset += 3;
    for (int i = 0; i < kJoints; ++i) {
        observation_(offset + i) = robot.joint_pos(policy_to_robot_[i]) - default_(i);
        observation_(offset + kJoints + i) = robot.joint_vel(policy_to_robot_[i]);
    }
    observation_.tail(kJoints) = previous_action_;
    const char* outputs[] = {"actions"};
    auto values = Infer(observation_, frame_, outputs, 1);
    VecXf action = Eigen::Map<const VecXf>(values[0].GetTensorData<float>(), kJoints);
    if (!action.allFinite()) throw std::runtime_error("Non-finite mimic action");
    RobotAction result;
    result.goal_joint_pos = ToRobot(default_ + scale_.cwiseProduct(action));
    if (!result.goal_joint_pos.allFinite()) throw std::runtime_error("Non-finite mimic target");
    result.goal_joint_vel = VecXf::Zero(kJoints);
    result.tau_ff = VecXf::Zero(kJoints);
    result.kp = JointKp();
    result.kd = JointKd();
    previous_action_ = action; // Raw action, no AMP clipping/scaling/history.
    if (advance) {
        if (frame_ + 1 < static_cast<int>(references_.size())) ++frame_;
        else holding_last_frame_ = true;
    }
    ++run_cnt_;
    return result;
}

void Sit2StandMimicPolicyRunner::ValidateTraining(const std::filesystem::path& directory) {
    std::ifstream file(directory / "motion.json");
    nlohmann::json manifest;
    file >> manifest;
    const auto& training = manifest.at("training");
    if (std::abs(training.at("control_period").get<double>() - .02) > 1e-9 ||
        !training.at("hold_last_frame").get<bool>())
        throw std::runtime_error("Sit2stand requires 50 Hz control and final-frame hold");
    for (const auto& item : std::vector<std::pair<const char*, VecXf>>{
             {"default_joint_pos", DefaultJointPosition()}, {"joint_stiffness", JointKp()},
             {"joint_damping", JointKd()}, {"action_scale", ActionScale()}}) {
        if ((TrainingVector(training, item.first) - item.second).cwiseAbs().maxCoeff() > 1e-6f)
            throw std::runtime_error(std::string("Sit2stand model/training mismatch: ") + item.first);
    }
    effort_limits_ = TrainingVector(training, "effort_limit");
    if ((effort_limits_.array() <= 0).any()) throw std::runtime_error("Invalid sit2stand effort limits");
}

std::filesystem::path Sit2StandMimicPolicyRunner::DefaultPolicyDirectory() {
    if (const char* path = std::getenv("SIT2STAND_MIMIC_POLICY_DIR"); path && *path) return path;
#ifdef MIMIC_POLICY_SOURCE_DIR
    const std::filesystem::path source =
        std::filesystem::path(MIMIC_POLICY_SOURCE_DIR) / "sit2stand_policy";
    if (std::filesystem::exists(source / "motion.json")) return source;
#endif
    return "/opt/robot/share/motion_master/policy/mimic/sit2stand_policy";
}
