#include "ysc_policy_runner.h"

#include <fstream>
#include <onnxruntime_cxx_api.h>

YscPolicyRunner::YscPolicyRunner(const std::string& policy_name, RobotName robot_name,
                                 const std::string& onnx_path)
    : PolicyRunnerBase(policy_name, robot_name), env_(ORT_LOGGING_LEVEL_ERROR, policy_name.c_str()),
      onnx_path_(onnx_path) {
    if (robot_name != RobotName::CR1STANDARD) {
        throw std::runtime_error("policy_CR1_8.onnx supports CR1STANDARD (21 DOF) only");
    }

    const std::vector<std::string> robot_order = {
        "waist_z_joint", "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint",
        "left_elbow_joint", "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint",
        "right_elbow_joint", "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint", "left_knee_joint",
        "left_ankle_y_joint", "left_ankle_x_joint", "right_hip_y_joint", "right_hip_x_joint",
        "right_hip_z_joint", "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint"};
    const std::vector<std::string> policy_order = {
        "left_hip_y_joint", "right_hip_y_joint", "waist_z_joint", "left_hip_x_joint", "right_hip_x_joint",
        "left_shoulder_y_joint", "right_shoulder_y_joint", "left_hip_z_joint", "right_hip_z_joint",
        "left_shoulder_x_joint", "right_shoulder_x_joint", "left_knee_joint", "right_knee_joint",
        "left_shoulder_z_joint", "right_shoulder_z_joint", "left_ankle_y_joint", "right_ankle_y_joint",
        "left_elbow_joint", "right_elbow_joint", "left_ankle_x_joint", "right_ankle_x_joint"};
    const std::vector<std::string> arm_joint_order = {
        "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint", "left_elbow_joint",
        "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint", "right_elbow_joint"};
    robot_to_policy_ = GeneratePermutation(robot_order, policy_order);
    policy_to_robot_ = GeneratePermutation(policy_order, robot_order);
    policy_to_arm_ = GeneratePermutation(policy_order, arm_joint_order);

    kp_.resize(action_dim_);
    kp_ << 300.f, 300.f, 200.f, 300.f, 300.f, 100.f, 100.f, 300.f, 300.f, 100.f, 100.f,
           300.f, 300.f, 100.f, 100.f, 80.f, 80.f, 100.f, 100.f, 30.f, 30.f;
    kd_.resize(action_dim_);
    kd_ << 10.f, 10.f, 10.f, 10.f, 10.f, 5.f, 5.f, 10.f, 10.f, 5.f, 5.f,
           10.f, 10.f, 5.f, 5.f, 3.f, 3.f, 5.f, 5.f, 1.f, 1.f;
    dof_pos_default_.resize(action_dim_);
    dof_pos_default_ << -0.4f, -0.4f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f,
                        0.f, 0.f, 0.8f, 0.8f, 0.f, 0.f, -0.4f, -0.4f,
                        0.5f, 0.5f, 0.f, 0.f;
    max_cmd_vel_ << 1.0f, 0.5f, 2.0f;
    cmd_vel_scale_ << 2.0f, 2.0f, 0.25f;
    decimation_ = 20;

    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::ifstream policy_file(onnx_path_);
    if (!policy_file.good()) throw std::runtime_error("YSC ONNX not found: " + onnx_path_);
    session_ = Ort::Session(env_, onnx_path_.c_str(), session_options_);

    const int input_dim = obs_dim_ * history_frames_;
    observation_.setZero(obs_dim_);
    observation_history_.setZero(obs_dim_ * (history_frames_ - 1));
    model_input_.setZero(input_dim);
    action_.setZero(action_dim_);
    last_action_.setZero(action_dim_);
    joint_pos_.setZero(action_dim_);
    joint_vel_.setZero(action_dim_);
    std::cout << "loaded YSC ONNX model from: " << onnx_path_ << std::endl;
}

YscPolicyRunner::~YscPolicyRunner() { std::cout << "YscPolicyRunner destroyed" << std::endl; }

std::vector<int> YscPolicyRunner::GeneratePermutation(const std::vector<std::string>& from,
                                                       const std::vector<std::string>& to) const {
    std::vector<int> permutation;
    for (const auto& name : to) {
        const auto it = std::find(from.begin(), from.end(), name);
        if (it == from.end()) throw std::runtime_error("YSC joint mapping missing: " + name);
        permutation.push_back(static_cast<int>(std::distance(from.begin(), it)));
    }
    return permutation;
}

void YscPolicyRunner::DisplayPolicyInfo() {
    std::cout << policy_name_ << ": state[1,432] -> output[1,21]" << std::endl;
}

void YscPolicyRunner::OnEnter() {
    observation_history_.setZero();
    model_input_.setZero();
    action_.setZero();
    last_action_.setZero();
    cmd_vel_input_.setZero();
    run_cnt_ = 0;
    run_cnt_record_ = 0;
}

void YscPolicyRunner::OnExit() {}

RobotAction YscPolicyRunner::GetRobotAction(const RobotBasicState& ro, const UserCommand& uc) {
    Vec3f requested_velocity(uc.forward_vel_scale, uc.side_vel_scale, uc.turnning_vel_scale);
    if (uc.forward_vel_scale < 0.f) requested_velocity[0] = 0.7f * uc.forward_vel_scale;
    if (uc.safe_control_mode != 0) run_cnt_record_ = run_cnt_;
    if (run_cnt_ - run_cnt_record_ < 100) requested_velocity.setZero();
    const Vec3f velocity_delta = requested_velocity - cmd_vel_input_;
    vel_delta_const_ << 0.02f, 0.02f, 0.04f;
    for (int i = 0; i < 3; ++i) {
        cmd_vel_input_(i) += std::max(-vel_delta_const_(i), std::min(vel_delta_const_(i), velocity_delta(i)));
    }

    for (int i = 0; i < action_dim_; ++i) {
        joint_pos_(i) = ro.joint_pos(robot_to_policy_[i]);
        joint_vel_(i) = ro.joint_vel(robot_to_policy_[i]);
    }
    const Vec3f projected_gravity = ro.base_rot_mat.transpose() * Vec3f(0.f, 0.f, -1.f);
    observation_ << omega_scale_ * ro.base_omega, projected_gravity,
                    cmd_vel_input_.cwiseProduct(max_cmd_vel_).cwiseProduct(cmd_vel_scale_), joint_pos_ - dof_pos_default_,
                    dof_vel_scale_ * joint_vel_, last_action_;
    observation_history_.head(obs_dim_ * (history_frames_ - 1)) =
        observation_history_.segment(obs_dim_, obs_dim_ * (history_frames_ - 1)).eval();
    observation_history_.tail(obs_dim_) = observation_;
    // policy_CR1_8 follows the original runner layout: current observation
    // first, followed by the five-frame history buffer.
    model_input_.head(obs_dim_) = observation_;
    model_input_.tail(obs_dim_ * (history_frames_ - 1)) = observation_history_;

    const std::vector<int64_t> shape = {1, obs_dim_ * history_frames_};
    auto tensor = Ort::Value::CreateTensor<float>(memory_info_, model_input_.data(),
                                                   model_input_.size(), shape.data(), shape.size());
    ort_inputs_.clear();
    ort_inputs_.push_back(std::move(tensor));
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, ort_inputs_.data(), ort_inputs_.size(),
                                output_names_, 1);
    action_ = Eigen::Map<VecXf>(outputs[0].GetTensorMutableData<float>(), action_dim_);
    last_action_ = action_;

    RobotAction robot_action;
    robot_action.goal_joint_pos.setZero(action_dim_);
    robot_action.goal_joint_vel.setZero(action_dim_);
    robot_action.tau_ff.setZero(action_dim_);
    robot_action.kp.setZero(action_dim_);
    robot_action.kd.setZero(action_dim_);
    for (int robot_index = 0; robot_index < action_dim_; ++robot_index) {
        const int policy_index = policy_to_robot_[robot_index];
        robot_action.goal_joint_pos(robot_index) =
            action_scale_ * action_(policy_index) + dof_pos_default_(policy_index);
        robot_action.kp(robot_index) = kp_(policy_index);
        robot_action.kd(robot_index) = kd_(policy_index);
    }
    ++run_cnt_;
    if (run_cnt_ < 100) {
        for (int arm_index = 0; arm_index < 8; ++arm_index) {
            const int policy_index = policy_to_arm_[arm_index];
            float delta = dof_pos_default_(policy_index) - joint_pos_(policy_index);
            if (std::fabs(delta) > 0.08f) delta = delta > 0.f ? 0.08f : -0.08f;
            robot_action.goal_joint_pos(1 + arm_index) = joint_pos_(policy_index) + delta;
        }
    }
    return robot_action;
}
