#include "knee_mimic_policy_runner.h"
#include <sstream>

namespace {
const char* inputs[] = {"obs", "time_step"};
const char* outputs[] = {"actions", "joint_pos", "joint_vel", "body_pos_w",
                         "body_quat_w", "body_lin_vel_w", "body_ang_vel_w"};
const char* joint_order =
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

std::string Metadata(const Ort::ModelMetadata& metadata, const char* key) {
    Ort::AllocatorWithDefaultOptions allocator;
    auto value = metadata.LookupCustomMetadataMapAllocated(key, allocator);
    if (!value) throw std::runtime_error(std::string("Missing knee policy metadata: ") + key);
    return value.get();
}
VecXf Parameters(const Ort::ModelMetadata& metadata, const char* key, bool positive) {
    std::istringstream stream(Metadata(metadata, key));
    std::string item;
    VecXf value(29);
    int count = 0;
    while (std::getline(stream, item, ',')) {
        if (count == 29) throw std::runtime_error("Too many knee policy parameters");
        size_t consumed = 0;
        value(count++) = std::stof(item, &consumed);
        if (consumed != item.size()) throw std::runtime_error("Invalid knee policy parameter");
    }
    if (count != 29 || !value.allFinite() || (positive && (value.array() <= 0).any()))
        throw std::runtime_error(std::string("Invalid knee policy metadata: ") + key);
    return value;
}
}

KneeMimicPolicyRunner::KneeMimicPolicyRunner(const std::string& name, RobotName robot,
        const std::string& relative_path, int last_frame, const std::string& override_path,
        bool independent_terminal_arms, UdpNode* udp)
    : PolicyRunnerBase(name, robot), last_frame_(last_frame),
      independent_terminal_arms_(independent_terminal_arms), udp_(udp) {
    decimation_ = 20;
    if (independent_terminal_arms_) {
        constexpr float half_pi = 1.57079632679f;
        // SDK elbow zero is bent: +pi/2 straightens the forearm in the B2 model.
        const Vec7f forward = (Vec7f() << -half_pi,0.f,0.f,half_pi,0.f,0.f,0.f).finished();
        const Vec7f left_spread = (Vec7f() << 0.f,half_pi,0.f,half_pi,0.f,0.f,0.f).finished();
        const Vec7f right_spread = (Vec7f() << 0.f,-half_pi,0.f,half_pi,0.f,0.f,0.f).finished();
        arms_.SetPresetTargets(1, forward, forward);
        arms_.SetPresetTargets(2, left_spread, right_spread);
    }
    try {
        path_ = override_path.empty() ? pro_policy::ResolveModelPath(relative_path) : override_path;
        options_.SetIntraOpNumThreads(1);
        options_.SetInterOpNumThreads(1);
        options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        session_ = Ort::Session(env_, path_.c_str(), options_);
        pro_policy::ValidateCopiedModel(session_, 151, true);
        const auto metadata = session_.GetModelMetadata();
        if (Metadata(metadata, "joint_names") != joint_order ||
            Metadata(metadata, "observation_names") !=
                "command,projected_gravity,base_ang_vel,joint_pos,joint_vel,actions" ||
            Metadata(metadata, "anchor_body_name") != "base_link" ||
            Metadata(metadata, "fps") != "50.0")
            throw std::runtime_error("Knee policy observation/joint order or frequency mismatch");
        defaults_ = Parameters(metadata, "default_joint_pos", false);
        scale_ = Parameters(metadata, "action_scale", true);
        kp_ = Parameters(metadata, "joint_stiffness", true);
        kd_ = Parameters(metadata, "joint_damping", true);
        model_loaded_ = true;
        OnEnter();
    } catch (const std::exception& error) {
        model_loaded_ = false;
        SetInferenceFault(error.what());
    }
}

std::vector<Ort::Value> KneeMimicPolicyRunner::Infer(VecXf& observation, int frame) {
    pro_policy::ValidateObservation(observation, 151);
    const int64_t shape[]{1, 151}, time_shape[]{1, 1};
    float time = static_cast<float>(frame);
    std::vector<Ort::Value> tensors;
    tensors.push_back(Ort::Value::CreateTensor<float>(memory_, observation.data(), 151, shape, 2));
    tensors.push_back(Ort::Value::CreateTensor<float>(memory_, &time, 1, time_shape, 2));
    auto output = session_.Run(Ort::RunOptions{nullptr}, inputs, tensors.data(), 2, outputs, 7);
    pro_policy::ValidateMimicOutputs(output);
    return output;
}

void KneeMimicPolicyRunner::CacheReference(const std::vector<Ort::Value>& output) {
    reference_pos_ = Eigen::Map<const VecXf>(output[1].GetTensorData<float>(), 29);
    reference_vel_ = Eigen::Map<const VecXf>(output[2].GetTensorData<float>(), 29);
}

void KneeMimicPolicyRunner::OnEnter() {
    ResetInferenceFault();
    if (!model_loaded_) throw std::runtime_error("Knee mimic model did not load: " + path_);
    run_cnt_ = 0;
    last_action_.setZero();
    terminal_arms_started_ = false;
    arms_.Reset(Vec7f::Zero(), Vec7f::Zero(), false);
    VecXf zero = VecXf::Zero(151);
    CacheReference(Infer(zero, 0));
}

void KneeMimicPolicyRunner::OnExit() {
    run_cnt_ = 0;
    last_action_.setZero();
    terminal_arms_started_ = false;
    arms_.Reset(Vec7f::Zero(), Vec7f::Zero(), false);
}

RobotAction KneeMimicPolicyRunner::ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) {
    VecXf q(29), dq(29), observation(151);
    for (int i = 0; i < 29; ++i) {
        q(i) = state.joint_pos(policy_to_robot_[i]);
        dq(i) = state.joint_vel(policy_to_robot_[i]);
    }
    const Vec3f gravity = state.base_rot_mat.transpose() * Vec3f(0.f, 0.f, -1.f);
    observation << reference_pos_, reference_vel_, gravity, state.base_omega,
                   q - defaults_, dq, last_action_;
    // Switch on the first inference USING the terminal reference (frame 383),
    // rather than the previous inference that merely prefetched that frame.
    const bool independent_arms = independent_terminal_arms_ && run_cnt_ == last_frame_;
    if (independent_arms) {
        for (int i = 0; i < 29; ++i) {
            if (policy_to_robot_[i] < 3 || policy_to_robot_[i] >= 17) continue;
            // Keep the exported motion command: replacing a reference pose
            // changes the task supplied to the waist/leg policy.
            observation(64 + i) = 0.f;           // Actual arm position - default.
            observation(93 + i) = 0.f;           // Actual arm velocity.
            observation(122 + i) = 0.f;          // Previous policy arm action.
        }
    }
    // The actor uses only obs. Fetch the next reference with this inference so
    // the next tick sees its own frame, with no one-frame lag or extra inference.
    const int next_frame = std::min(run_cnt_ + 1, last_frame_);
    const auto output = Infer(observation, next_frame);
    last_action_ = Eigen::Map<const VecXf>(output[0].GetTensorData<float>(), 29);
    if (independent_arms) {
        for (int i = 0; i < 29; ++i)
            if (policy_to_robot_[i] >= 3 && policy_to_robot_[i] < 17) last_action_(i) = 0.f;
    }
    CacheReference(output);
    run_cnt_ = next_frame;
    RobotAction action;
    action.goal_joint_pos = VecXf::Zero(29);
    action.goal_joint_vel = VecXf::Zero(29);
    action.kp = VecXf::Zero(29);
    action.kd = VecXf::Zero(29);
    action.tau_ff = VecXf::Zero(29);
    for (int i = 0; i < 29; ++i) {
        const int robot = policy_to_robot_[i];
        action.goal_joint_pos(robot) = defaults_(i) + scale_(i) * last_action_(i);
        action.kp(robot) = kp_(i);
        action.kd(robot) = kd_(i);
    }
    if (independent_arms) {
        if (!terminal_arms_started_) {
            // Seed the independent controller from real feedback. Frozen model
            // observations never replace the feedback used for arm control/PD.
            arms_.Reset(state.joint_pos.segment<7>(3), state.joint_pos.segment<7>(10), true);
            terminal_arms_started_ = true;
        }
        action.goal_joint_pos.segment(3, 14) = arms_.Update(state, command, .02f, udp_);
        // Same independent arm gains as RLStandPolicyRunner, in SDK order.
        const Vec7f arm_kp = (Vec7f() << 300.f,100.f,100.f,100.f,40.f,40.f,40.f).finished();
        const Vec7f arm_kd = (Vec7f() << 15.f,3.f,3.f,3.f,2.f,2.f,2.f).finished();
        action.kp.segment<7>(3) = action.kp.segment<7>(10) = arm_kp;
        action.kd.segment<7>(3) = action.kd.segment<7>(10) = arm_kd;
    }
    return action;
}

void KneeMimicPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << ": 151 observations / 29 actions, 50 Hz, "
              << last_frame_ + 1 << " frames, terminal reference held; " << path_ << std::endl;
    if (independent_terminal_arms_)
        std::cout << "Terminal arms: default-pose observations, zero policy arm actions; RL Stand presets/UDP" << std::endl;
}
