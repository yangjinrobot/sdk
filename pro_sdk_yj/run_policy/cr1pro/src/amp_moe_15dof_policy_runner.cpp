#include "amp_moe_15dof_policy_runner.hpp"

RobotAction AmpMoe15DofPolicyRunner::ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) {
    Vec3f requested(command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale);
    requested = requested.cwiseMax(-1.f).cwiseMin(1.f);
    const Vec3f increments(.02f, .02f, .04f);
    for (int i = 0; i < 3; ++i)
        cmd_vel_input_(i) += std::clamp(requested(i) - cmd_vel_input_(i), -increments(i), increments(i));

    VecXf frame(kFrame);
    frame.segment<3>(0) = state.base_omega;
    frame.segment<3>(3) = state.base_rot_mat.transpose() * Vec3f(0, 0, -1);
    frame.segment<3>(6) =
        ApplyCommandDeadband(cmd_vel_input_.cwiseProduct(max_cmd_vel_));
    for (int i = 0; i < 29; ++i) {
        const int joint = kObservationJoints[i];
        frame(9 + i) = state.joint_pos(joint) - dof_pos_default_(joint);
        frame(38 + i) = .05f * state.joint_vel(joint);
        frame(67 + i) = last_action_(joint);
    }
    pro_policy::ValidateObservation(frame, kFrame);
    frame = frame.cwiseMax(-100.f).cwiseMin(100.f);
    // simtosim.py: repeat the first observation five times; thereafter
    // append current before stacking. The last history frame equals current.
    if (obs_history_deque.empty()) {
        obs_history_deque.assign(kHistory, frame);
    } else {
        obs_history_deque.pop_front();
        obs_history_deque.push_back(frame);
    }
    for (int i = 0; i < kHistory; ++i) observation_total_.segment(i * kFrame, kFrame) = obs_history_deque[i];
    observation_total_.tail(kFrame) = frame;
    pro_policy::ValidateObservation(observation_total_, kInput);
    const std::array<int64_t, 2> shape{1, kInput};
    auto tensor = Ort::Value::CreateTensor<float>(memory_info_, observation_total_.data(), observation_total_.size(), shape.data(), shape.size());
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, &tensor, 1, output_names_, 1);
    VecXf action = pro_policy::ReadAction(outputs, kActions);
    action = action.cwiseMax(-100.f).cwiseMin(100.f);

    RobotAction result;
    result.goal_joint_pos = dof_pos_default_;
    result.goal_joint_vel = VecXf::Zero(29);
    result.tau_ff = VecXf::Zero(29);
    result.kp = kp_;
    result.kd = kd_;
    last_action_.setZero();  // Externally planned arms have zero policy action.
    for (int i = 0; i < kActions; ++i) {
        const int joint = kActionJoints[i];
        last_action_(joint) = action(i);
        result.goal_joint_pos(joint) += .25f * action(i);
    }
    result.goal_joint_pos.segment(3, 14) = arms_.Update(state, command, kDt, udp_);
    ++run_cnt_;
    return result;
}

AmpMoe15DofPolicyRunner::AmpMoe15DofPolicyRunner(const std::string& policy_name, RobotName robot_name)
    : PolicyRunnerBase(policy_name, robot_name) {
    decimation_ = 20;
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
        onnx_path_ = pro_policy::ResolveModelPath("amp_moe_policy/moe_policy_1026.onnx");
        session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
        pro_policy::ValidateModelSchema(session_, kInput, kActions, input_names_[0], output_names_[0]);
        model_loaded_ = true;
        DisplayPolicyInfo();
    } catch (const std::exception& error) { SetInferenceFault(error.what()); }
}

void AmpMoe15DofPolicyRunner::SetUdpNode(UdpNode* node) { udp_ = node; }

UdpNode* AmpMoe15DofPolicyRunner::GetUdpNode() { return udp_; }

bool AmpMoe15DofPolicyRunner::IsReady() const { return model_loaded_ && !HasInferenceFault(); }

void AmpMoe15DofPolicyRunner::OnEnter() {
    if (model_loaded_) ResetInferenceFault();
    last_action_.setZero();
    observation_total_.setZero();
    obs_history_deque.clear();
    cmd_vel_input_.setZero();
    run_cnt_ = 0;
    const Vec7f left = initial_arms_pending_ ? initial_left_ : Vec7f(dof_pos_default_.segment<7>(3));
    const Vec7f right = initial_arms_pending_ ? initial_right_ : Vec7f(dof_pos_default_.segment<7>(10));
    arms_.Reset(left, right, initial_arms_pending_);
    initial_arms_pending_ = false;
}

void AmpMoe15DofPolicyRunner::OnExit() {}

void AmpMoe15DofPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << ": " << onnx_path_ << " observations[1,576] -> actions[1,15], 50 Hz\n";
}

void AmpMoe15DofPolicyRunner::SetDecimation(int value) {
    if (value != 20) throw std::invalid_argument("AMP MoE requires a 20 ms policy period");
    PolicyRunnerBase::SetDecimation(value);
}

void AmpMoe15DofPolicyRunner::SetCmdMaxVel(const Vec3f& velocity) {
    if (!velocity.allFinite() || (velocity.array() < 0.f).any())
        throw std::invalid_argument("AMP MoE velocity limits must be finite and nonnegative");
    max_cmd_vel_ = velocity.cwiseMin(Vec3f(.8f, .8f, 1.57f));
}

void AmpMoe15DofPolicyRunner::SetInitialArmTargets(const Vec7f& left, const Vec7f& right) {
    if (!left.allFinite() || !right.allFinite()) throw std::invalid_argument("non-finite initial arm targets");
    initial_left_ = left; initial_right_ = right; initial_arms_pending_ = true;
}

void AmpMoe15DofPolicyRunner::SetArmInterpDuration(float duration) { arms_.SetDuration(duration); }

const VecXf& AmpMoe15DofPolicyRunner::GetDefaultJointPosition() const { return dof_pos_default_; }
