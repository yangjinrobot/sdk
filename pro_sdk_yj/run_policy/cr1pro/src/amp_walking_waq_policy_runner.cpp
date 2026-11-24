#include "amp_walking_waq_policy_runner.hpp"

RobotAction AmpWalkingWAQPolicyRunner::ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) {
    Vec3f requested(command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale);
    requested = requested.cwiseMax(-1.f).cwiseMin(1.f);
    const Vec3f increments(.02f, .02f, .04f);
    for (int i = 0; i < 3; ++i)
        cmd_vel_input_(i) += std::clamp(requested(i) - cmd_vel_input_(i), -increments(i), increments(i));

    VecXf frame(kFrame);
    frame.segment<3>(0) = state.base_omega;
    frame.segment<3>(3) = state.base_rot_mat.transpose() * Vec3f(0, 0, -1);
    // Match the SDK velocity API while respecting asymmetric deployment limits.
    Vec3f velocity = cmd_vel_input_.cwiseProduct(max_cmd_vel_);
    for (int i = 0; i < 3; ++i) {
        const float lower = std::min(0.0f, min_cmd_vel_(i));
        const float upper = std::max(0.0f, max_cmd_vel_(i));
        velocity(i) = std::clamp(velocity(i), lower, upper);
    }
    frame.segment<3>(6) = ApplyCommandDeadband(velocity);
    for (int i = 0; i < kActions; ++i) {
        const int joint = kJoints[i];
        frame(9 + i) = state.joint_pos(joint) - dof_pos_default_(joint);
        frame(38 + i) = .05f * state.joint_vel(joint);
        frame(67 + i) = last_action_(i);
    }
    pro_policy::ValidateObservation(frame, kFrame);
    frame = frame.cwiseMax(-100.f).cwiseMin(100.f);
    // Oldest -> newest, including current; fill initial history with the
    // entry observation. This ActorCritic consumes only these five frames.
    if (obs_history_deque.empty()) {
        obs_history_deque.assign(kHistory, frame);
    } else {
        obs_history_deque.pop_front();
        obs_history_deque.push_back(frame);
    }
    for (int i = 0; i < kHistory; ++i) observation_total_.segment(i * kFrame, kFrame) = obs_history_deque[i];
    pro_policy::ValidateObservation(observation_total_, kInput);
    const std::array<int64_t, 2> shape{1, kInput};
    auto tensor = Ort::Value::CreateTensor<float>(memory_info_, observation_total_.data(), observation_total_.size(), shape.data(), shape.size());
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, &tensor, 1, output_names_, 1);
    // The environment observes the raw action-buffer entry; clipping is
    // applied separately to position targets and to the observation frame.
    last_action_ = pro_policy::ReadAction(outputs, kActions);
    const VecXf action = last_action_.cwiseMax(-100.f).cwiseMin(100.f);

    RobotAction result;
    result.goal_joint_pos = dof_pos_default_;
    result.goal_joint_vel = VecXf::Zero(29);
    result.tau_ff = VecXf::Zero(29);
    result.kp = kp_;
    result.kd = kd_;
    for (int i = 0; i < kActions; ++i)
        result.goal_joint_pos(kJoints[i]) += .25f * action(i);
    ++run_cnt_;
    return result;
}

AmpWalkingWAQPolicyRunner::AmpWalkingWAQPolicyRunner(const std::string& policy_name, RobotName robot_name)
    : PolicyRunnerBase(policy_name, robot_name) {
    decimation_ = 20;
    // Default joint positions and actuator gains from walktorun/env(15).yaml,
    // in SDK order (waist / left arm / right arm / left leg / right leg).
    dof_pos_default_ << 0, 0, 0,
        0, .2f, 0, .8f, 0, 0, 0, 0, -.2f, 0, .8f, 0, 0, 0,
        -.15f, 0, 0, .3f, -.15f, 0, -.15f, 0, 0, .3f, -.15f, 0;
    kp_ << 350, 350, 350,
        200, 150, 150, 200, 150, 150, 150, 200, 150, 150, 200, 150, 150, 150,
        350, 350, 200, 350, 100, 100, 350, 350, 200, 350, 100, 100;
    kd_ << 15, 15, 15,
        5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
        10, 10, 5, 10, 5, 5, 10, 10, 5, 10, 5, 5;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    try {
        onnx_path_ = pro_policy::ResolveModelPath("amp_policy/walktorun/policy_1030.onnx");
        session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
        pro_policy::ValidateModelSchema(session_, kInput, kActions, input_names_[0], output_names_[0]);
        model_loaded_ = true;
    } catch (const std::exception& error) { SetInferenceFault(error.what()); }
}

bool AmpWalkingWAQPolicyRunner::IsReady() const { return model_loaded_ && !HasInferenceFault(); }

void AmpWalkingWAQPolicyRunner::OnEnter() {
    if (model_loaded_) ResetInferenceFault();
    last_action_.setZero();
    observation_total_.setZero();
    obs_history_deque.clear();
    cmd_vel_input_.setZero();
    run_cnt_ = 0;
}

void AmpWalkingWAQPolicyRunner::OnExit() {}

void AmpWalkingWAQPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << ": " << onnx_path_ << " obs[1,480] -> actions[1,29], 50 Hz\n";
}

void AmpWalkingWAQPolicyRunner::SetDecimation(int value) {
    if (value != 20) throw std::invalid_argument("AMP Walking WAQ requires a 20 ms policy period");
    PolicyRunnerBase::SetDecimation(value);
}

void AmpWalkingWAQPolicyRunner::SetCmdMaxVel(const Vec3f& velocity) {
    if (!velocity.allFinite() || (velocity.array() < 0.f).any())
        throw std::invalid_argument("AMP Walking WAQ velocity limits must be finite and nonnegative");
    max_cmd_vel_ = velocity.cwiseMin(max_velocity_limit_);
    min_cmd_vel_ = (-max_cmd_vel_).cwiseMax(min_velocity_limit_);
}

const VecXf& AmpWalkingWAQPolicyRunner::GetDefaultJointPosition() const { return dof_pos_default_; }
