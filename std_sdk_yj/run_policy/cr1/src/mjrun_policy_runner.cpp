#include "mjrun_policy_runner.h"

#include <algorithm>
#include <array>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {
std::vector<int> GeneratePermutation(const std::vector<std::string>& from,
                                     const std::vector<std::string>& to) {
    std::vector<int> indices;
    indices.reserve(to.size());
    for (const auto& name : to) {
        const auto match = std::find(from.begin(), from.end(), name);
        if (match == from.end()) {
            throw std::logic_error("Unknown mjrun joint: " + name);
        }
        indices.push_back(static_cast<int>(match - from.begin()));
    }
    return indices;
}
}  // namespace

MjRunPolicyRunner::MjRunPolicyRunner(RobotName robot, const std::string& path)
    : PolicyRunnerBase("mjrun", robot), onnx_path_(path) {
    if (robot != RobotName::CR1STANDARD) throw std::invalid_argument("mjrun requires CR1STANDARD");
    decimation_ = 20;
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1)
        throw std::runtime_error("mjrun expects one input and one output");
    Ort::AllocatorWithDefaultOptions allocator;
    const auto input_name = session_.GetInputNameAllocated(0, allocator);
    const auto output_name = session_.GetOutputNameAllocated(0, allocator);
    const auto input_type = session_.GetInputTypeInfo(0);
    const auto output_type = session_.GetOutputTypeInfo(0);
    const auto input = input_type.GetTensorTypeAndShapeInfo();
    const auto output = output_type.GetTensorTypeAndShapeInfo();
    if (std::string(input_name.get()) != "obs" || std::string(output_name.get()) != "actions" ||
        input.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        output.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        input.GetShape() != std::vector<int64_t>{1, kInputSize} ||
        output.GetShape() != std::vector<int64_t>{1, kJoints})
        throw std::runtime_error("mjrun requires float obs[1,360] and actions[1,21]");

    // SDK order and model order for the 21 Standard joints.
    const std::vector<std::string> robot_order{
            "waist_yaw_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_pitch_joint",
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"
    };
    const std::vector<std::string> policy_order{
            "waist_yaw_joint",
            "left_shoulder_pitch_joint", "left_shoulder_roll_joint",
            "left_shoulder_yaw_joint", "left_elbow_pitch_joint",
            "right_shoulder_pitch_joint", "right_shoulder_roll_joint",
            "right_shoulder_yaw_joint", "right_elbow_pitch_joint",
            "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
            "left_knee_pitch_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
            "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
            "right_knee_pitch_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"
    };
    robot2policy_idx = GeneratePermutation(robot_order, policy_order);
    policyandpd2robot_idx = GeneratePermutation(policy_order, robot_order);

    // Source: env.yaml scene.entities.robot.init_state.joint_pos.
    dof_pos_default_.resize(kJoints);
    dof_pos_default_ << 0.0f, 0.0f, 0.15f, 0.0f, 1.2f,
                         0.0f, -0.15f, 0.0f, 1.2f, -0.2f,
                         0.0f, 0.0f, 0.4f, -0.2f, 0.0f,
                         -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f;
    // Source: env.yaml actuator groups, matched to policy_order.
    kp_.resize(kJoints);
    kd_.resize(kJoints);
    kp_ << 200, 100, 100, 100, 100, 100, 100, 100, 100,
           300, 300, 200, 300, 80, 30, 300, 300, 200, 300, 80, 30;
    kd_ << 5, 5, 5, 5, 5, 5, 5, 5, 5,
           10, 10, 5, 10, 3, 1, 10, 10, 5, 10, 3, 1;

    // Warm up the ONNX session before measuring inference in the control loop.
    RobotBasicState standing{};
    standing.base_rot_mat.setIdentity();
    standing.base_omega.setZero();
    standing.joint_pos = dof_pos_default_;
    standing.joint_vel = JointVector::Zero();
    OnEnter();
    for (int i = 0; i < 10; ++i) {
        GetRobotAction(standing, UserCommand{});
    }
    OnExit();
    std::cout << "loaded " << policy_name_ << " ONNX model from: " << onnx_path_ << std::endl;
}

void MjRunPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << " network test success" << std::endl;
    std::cout << "dim  : " << kFrameSize << " " << kHistoryLength << " " << kInputSize
              << " " << kJoints << std::endl;
    std::cout << "dof  : " << dof_pos_default_.transpose() << std::endl;
    std::cout << "kp   : " << kp_.transpose() << std::endl;
    std::cout << "kd   : " << kd_.transpose() << std::endl;
    std::cout << "max_v: " << max_cmd_vel_.transpose() << std::endl;
}

void MjRunPolicyRunner::OnEnter() {
    cmd_vel_input_.setZero();
    previous_action_.setZero();
    seed_history_ = true;
}

void MjRunPolicyRunner::OnExit() {
    OnEnter();
}

void MjRunPolicyRunner::UpdateCommandVelocity(const Vec3f& axes) {
    const float forward = std::clamp(axes.x(), -1.0f, 1.0f);
    const float forward_target = forward * (forward < 0.0f ? 0.25f : max_cmd_vel_.x());
    const float forward_step = forward_acceleration_ * static_cast<float>(kPolicySeconds);
    cmd_vel_input_.x() += std::clamp(
        forward_target - cmd_vel_input_.x(), -forward_step, forward_step);
    cmd_vel_input_.y() = max_cmd_vel_.y() * std::clamp(axes.y(), -1.0f, 1.0f);
    cmd_vel_input_.z() = max_cmd_vel_.z() * std::clamp(axes.z(), -1.0f, 1.0f);
}

RobotAction MjRunPolicyRunner::GetRobotAction(const RobotBasicState& state, const UserCommand& command) {
    const Vec3f axes(command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale);
    if (!axes.allFinite() || !state.base_rot_mat.allFinite() || !state.base_omega.allFinite() ||
        state.joint_pos.size() != num_dof_ || state.joint_vel.size() != num_dof_ ||
        !state.joint_pos.allFinite() || !state.joint_vel.allFinite())
        throw std::runtime_error("Invalid mjrun feedback or joystick");

    UpdateCommandVelocity(axes);

    JointVector observation_joint_pos;
    JointVector observation_joint_vel;
    for (int index = 0; index < act_dim_; ++index) {
        observation_joint_pos(index) =
            state.joint_pos(robot2policy_idx[index]) - dof_pos_default_(index);
        observation_joint_vel(index) =
            dof_vel_scale_ * state.joint_vel(robot2policy_idx[index]);
    }

    Eigen::Matrix<float, kFrameSize, 1> frame;
    frame << state.base_omega, state.base_rot_mat.transpose() * Vec3f(0.f, 0.f, -1.f), cmd_vel_input_,
        observation_joint_pos, observation_joint_vel, previous_action_;
    if (!frame.allFinite()) throw std::runtime_error("Non-finite mjrun observation");
    frame = frame.cwiseMax(-observation_clip_).cwiseMin(observation_clip_);
    if (seed_history_) {
        for (int index = 0; index < kHistoryLength; ++index)
            std::copy_n(frame.data(), kFrameSize, history_.begin() + index * kFrameSize);
        seed_history_ = false;
    } else {
        std::copy(history_.begin() + kFrameSize, history_.end(), history_.begin());
        std::copy_n(frame.data(), kFrameSize, history_.end() - kFrameSize);
    }

    JointVector action;
    const std::array<int64_t, 2> input_shape{1, kInputSize};
    const std::array<int64_t, 2> output_shape{1, kJoints};
    auto input = Ort::Value::CreateTensor<float>(
        memory_info_, history_.data(), history_.size(), input_shape.data(), input_shape.size());
    auto output = Ort::Value::CreateTensor<float>(
        memory_info_, action.data(), kJoints, output_shape.data(), output_shape.size());
    session_.Run(Ort::RunOptions{nullptr}, input_names_, &input, 1, output_names_, &output, 1);
    if (!action.allFinite()) throw std::runtime_error("Non-finite mjrun action");

    RobotAction result;
    result.goal_joint_pos.resize(kJoints);
    result.goal_joint_vel = JointVector::Zero();
    result.kp.resize(kJoints);
    result.kd.resize(kJoints);
    result.tau_ff = JointVector::Zero();
    for (int index = 0; index < act_dim_; ++index) {
        const int policy_index = policyandpd2robot_idx[index];
        result.goal_joint_pos(index) = dof_pos_default_(policy_index) + action_scale_ * action(policy_index);
        result.kp(index) = kp_(policy_index);
        result.kd(index) = kd_(policy_index);
    }
    if (!result.goal_joint_pos.allFinite()) throw std::runtime_error("Non-finite mjrun target");
    previous_action_ = action;
    return result;
}
