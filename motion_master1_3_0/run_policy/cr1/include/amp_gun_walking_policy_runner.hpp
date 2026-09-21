#ifndef AMP_GUN_WALKING_POLICY_RUNNER_HPP_
#define AMP_GUN_WALKING_POLICY_RUNNER_HPP_

#include <array>
#include <algorithm>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <unordered_map>

#include "policy_runner_base.hpp"

class AmpGunWalkingPolicyRunner : public PolicyRunnerBase {
public:
    static constexpr int kDof = 21;
    static constexpr int kObsDim = 72;
    static constexpr int kHistoryLength = 5;
    static constexpr int kInputDim = kObsDim * (kHistoryLength + 1);
    // 调试时统一调整关节刚度比例。
    static constexpr float kKpScale = 1.0f;

    explicit AmpGunWalkingPolicyRunner(RobotName robot_name)
        : PolicyRunnerBase("amp_gun_walking", robot_name),
          env_(ORT_LOGGING_LEVEL_ERROR, "amp_gun_walking") {
        if (robot_name != RobotName::CR1STANDARD) {
            throw std::invalid_argument("AMP Gun Walking policy requires CR1_STANDARD");
        }

        const std::vector<std::string> robot_order = {
            "waist_z_joint", "left_shoulder_y_joint", "left_shoulder_x_joint", "left_shoulder_z_joint", "left_elbow_joint",
            "right_shoulder_y_joint", "right_shoulder_x_joint", "right_shoulder_z_joint", "right_elbow_joint",
            "left_hip_y_joint", "left_hip_x_joint", "left_hip_z_joint", "left_knee_joint", "left_ankle_y_joint", "left_ankle_x_joint",
            "right_hip_y_joint", "right_hip_x_joint", "right_hip_z_joint", "right_knee_joint", "right_ankle_y_joint", "right_ankle_x_joint"};
        const std::vector<std::string> policy_order = {
            "left_hip_y_joint", "right_hip_y_joint", "waist_z_joint", "left_hip_x_joint", "right_hip_x_joint",
            "left_shoulder_y_joint", "right_shoulder_y_joint", "left_hip_z_joint", "right_hip_z_joint",
            "left_shoulder_x_joint", "right_shoulder_x_joint", "left_knee_joint", "right_knee_joint",
            "left_shoulder_z_joint", "right_shoulder_z_joint", "left_ankle_y_joint", "right_ankle_y_joint",
            "left_elbow_joint", "right_elbow_joint", "left_ankle_x_joint", "right_ankle_x_joint"};
        policy_to_robot_ = BuildIndexMap(robot_order, policy_order);
        robot_to_policy_ = BuildIndexMap(policy_order, robot_order);

        VecXf robot_default(kDof), robot_kp(kDof), robot_kd(kDof);
        robot_default << 0.0f, 
                         0.0f, 0.20f, 0.0f, 0.0f, 
                         0.0f, -0.20f, 0.0f, 0.0f,
                         -0.10f, 0.0f, 0.0f, 0.20f, -0.10f, 0.0f, 
                         -0.10f, 0.0f, 0.0f, 0.20f, -0.10f, 0.0f;
        // 按腰、左臂、右臂、左腿、右腿排列。
        // robot_default << -0.0f,
        //                  -0.90f, 0.47f, -0.34f, 0.73f,
        //                  -0.48f, -0.45f, -0.15f, 0.20f,
        //                  -0.40f, 0.19f, 0.07f, 0.32f, -0.03f, -0.10f,
        //                  -0.24f, 0.02f, -0.12f, 0.46f, -0.27f, 0.01f;
        // b3_CFG stiffness/damping，按 robot_order 排列：腰、左臂、右臂、左腿、右腿。
        // 训练关节 pitch/roll/yaw 对应部署关节 y/x/z；kp 另乘上方调试倍率。
        robot_kp << 300,
                    200, 150, 150, 200,
                    200, 150, 150, 200,
                    300, 300, 200, 300, 100, 100,
                    300, 300, 200, 300, 100, 100;
        robot_kd << 15,
                    5, 5, 5, 5,
                    5, 5, 5, 5,
                    10, 10, 5, 10, 5, 5,
                    10, 10, 5, 10, 5, 5;
        default_pos_.resize(kDof); kp_.resize(kDof); kd_.resize(kDof);
        for (int policy_index = 0; policy_index < kDof; ++policy_index) {
            const int robot_index = policy_to_robot_[policy_index];
            default_pos_(policy_index) = robot_default(robot_index);
            kp_(policy_index) = kKpScale * robot_kp(robot_index);
            kd_(policy_index) = robot_kd(robot_index);
        }

        decimation_ = 20;
        max_cmd_vel_ = Vec3f(1.0f, 0.5f, 1.57f);
        observation_.setZero(kObsDim); policy_input_.setZero(kInputDim); action_.setZero(kDof);
        policy_joint_pos_.setZero(kDof); policy_joint_vel_.setZero(kDof);
        session_options_.SetIntraOpNumThreads(1);
        session_options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        const char* configured_path = std::getenv("AMP_GUN_WALKING_POLICY_PATH");
        if (configured_path && *configured_path) {
            policy_path_ = configured_path;
        } else {
            // Use this checkout's AMP model locally and the packaged model on the robot.
            const auto source_path = std::filesystem::path(__FILE__).parent_path().parent_path() /
                                     "policy/policy_0911_zjie.onnx";
            policy_path_ = std::filesystem::exists(source_path) ? source_path.string() :
                "/opt/robot/share/motion_master/policy/policy_0911_zjie.onnx";
        }
        if (!std::filesystem::is_regular_file(policy_path_))
            throw std::runtime_error("AMP Gun Walking model not found: " + policy_path_);
        session_ = Ort::Session(env_, policy_path_.c_str(), session_options_);
        std::cout << "loaded " << policy_name_ << " ONNX model from: "
                  << policy_path_ << std::endl;
        ValidateModelShape();
    }

    void DisplayPolicyInfo() override { std::cout << "AMP Gun Walking policy: " << policy_path_ << std::endl; }
    void OnEnter() override { observation_.setZero(); policy_input_.setZero(); action_.setZero(); history_.clear(); cmd_vel_input_.setZero(); }
    void OnExit() override {}

    RobotAction GetRobotAction(const RobotBasicState& robot, const UserCommand& command) override {
        if (robot.joint_pos.size() != kDof || robot.joint_vel.size() != kDof) throw std::runtime_error("AMP Gun Walking requires 21 joint observations");
        const Vec3f requested(command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale);
        Vec3f delta = requested - cmd_vel_input_;
        const Vec3f max_delta(0.02f, 0.02f, 0.04f);
        for (int index = 0; index < 3; ++index) delta(index) = std::clamp(delta(index), -max_delta(index), max_delta(index));
        cmd_vel_input_ += delta;
        for (int index = 0; index < kDof; ++index) { policy_joint_pos_(index) = robot.joint_pos(policy_to_robot_[index]); policy_joint_vel_(index) = robot.joint_vel(policy_to_robot_[index]); }
        const Vec3f projected_gravity = robot.base_rot_mat.transpose() * Vec3f(0.0f, 0.0f, -1.0f);
        observation_ << robot.base_omega, projected_gravity, cmd_vel_input_.cwiseProduct(max_cmd_vel_), policy_joint_pos_ - default_pos_, 0.05f * policy_joint_vel_, action_;
        if (!observation_.allFinite()) throw std::runtime_error("non-finite AMP observation");
        observation_ = observation_.cwiseMax(-100.0f).cwiseMin(100.0f);
        // history_ stores the five frames preceding the explicit current
        // frame. Updating it before packing would duplicate the current frame
        // and discard the oldest observation.
        if (history_.empty()) history_.assign(kHistoryLength, observation_);
        int offset = 0;
        for (const auto& frame : history_) { policy_input_.segment(offset, kObsDim) = frame; offset += kObsDim; }
        policy_input_.segment(offset, kObsDim) = observation_;
        history_.pop_front();
        history_.push_back(observation_);
        const std::array<int64_t, 2> input_shape = {1, kInputDim};
        auto input = Ort::Value::CreateTensor<float>(memory_info_, policy_input_.data(), policy_input_.size(), input_shape.data(), input_shape.size());
        const Ort::Value* input_ptr = &input;
        auto output = session_.Run(Ort::RunOptions{nullptr}, input_names_, input_ptr, 1, output_names_, 1);
        if (output.size() != 1 || !output[0].IsTensor() ||
            output[0].GetTensorTypeAndShapeInfo().GetElementCount() != kDof)
            throw std::runtime_error("invalid AMP action shape");
        action_ = Eigen::Map<VecXf>(output[0].GetTensorMutableData<float>(), kDof);
        if (!action_.allFinite()) throw std::runtime_error("non-finite AMP action");
        action_ = action_.cwiseMax(-100.0f).cwiseMin(100.0f);
        RobotAction result;
        result.goal_joint_pos.setZero(kDof); result.goal_joint_vel.setZero(kDof); result.tau_ff.setZero(kDof); result.kp.setZero(kDof); result.kd.setZero(kDof);
        for (int robot_index = 0; robot_index < kDof; ++robot_index) { const int policy_index = robot_to_policy_[robot_index]; result.goal_joint_pos(robot_index) = default_pos_(policy_index) + 0.25f * action_(policy_index); result.kp(robot_index) = kp_(policy_index); result.kd(robot_index) = kd_(policy_index); }
        return result;
    }

private:
    static std::vector<int> BuildIndexMap(const std::vector<std::string>& source, const std::vector<std::string>& target) {
        std::unordered_map<std::string, int> source_index;
        for (int index = 0; index < static_cast<int>(source.size()); ++index) source_index.emplace(source[index], index);
        std::vector<int> result; result.reserve(target.size());
        for (const auto& name : target) { const auto found = source_index.find(name); if (found == source_index.end()) throw std::runtime_error("unknown AMP joint: " + name); result.push_back(found->second); }
        return result;
    }
    void ValidateModelShape() {
        if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1)
            throw std::runtime_error("AMP Gun Walking requires one input and one output");
        Ort::AllocatorWithDefaultOptions allocator;
        input_name_ = session_.GetInputNameAllocated(0, allocator).get();
        output_name_ = session_.GetOutputNameAllocated(0, allocator).get();
        input_names_[0] = input_name_.c_str();
        output_names_[0] = output_name_.c_str();
        if (session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
            session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
            throw std::runtime_error("AMP Gun Walking requires float32 tensors");
        const auto input_shape = session_.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        const auto output_shape = session_.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1 || input_shape.size() != 2 || (input_shape[0] != 1 && input_shape[0] != -1) || input_shape[1] != kInputDim || output_shape.size() != 2 || (output_shape[0] != 1 && output_shape[0] != -1) || output_shape[1] != kDof) throw std::runtime_error("AMP Gun Walking model must be [batch,432] -> [batch,21]");
    }
    VecXf kp_, kd_, default_pos_, observation_, policy_input_, action_, policy_joint_pos_, policy_joint_vel_;
    Vec3f max_cmd_vel_;
    std::deque<VecXf> history_;
    std::vector<int> policy_to_robot_, robot_to_policy_;
    std::string policy_path_;
    Ort::Env env_;
    Ort::SessionOptions session_options_; Ort::Session session_{nullptr}; Ort::MemoryInfo memory_info_{nullptr};
    std::string input_name_, output_name_;
    const char* input_names_[1] = {"obs"};
    const char* output_names_[1] = {"actions"};
};

#endif
