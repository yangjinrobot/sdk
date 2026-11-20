#include "rl_stand_policy_runner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <stdexcept>

void RLStandPolicyRunner::ValidateModel() {
    if (session_.GetInputCount() != 1 || session_.GetOutputCount() != 1)
        throw std::runtime_error("restand: expected one input and one output");
    Ort::AllocatorWithDefaultOptions allocator;
    auto input = session_.GetInputNameAllocated(0, allocator);
    auto output = session_.GetOutputNameAllocated(0, allocator);
    if (std::string(input.get()) != input_names_[0] || std::string(output.get()) != output_names_[0])
        throw std::runtime_error("restand: expected obs/actions tensor names");
    auto input_type = session_.GetInputTypeInfo(0);
    auto output_type = session_.GetOutputTypeInfo(0);
    auto in = input_type.GetTensorTypeAndShapeInfo();
    auto out = output_type.GetTensorTypeAndShapeInfo();
    const auto in_shape = in.GetShape(), out_shape = out.GetShape();
    if (in.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        out.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        in_shape.size() != 2 || out_shape.size() != 2 ||
        (in_shape[0] != -1 && in_shape[0] != 1) ||
        (out_shape[0] != -1 && out_shape[0] != 1) ||
        in_shape[1] != 330 || out_shape[1] != 13)
        throw std::runtime_error("restand: expected float [batch,330] -> [batch,13]");
}

std::filesystem::path RLStandPolicyRunner::DefaultPolicyPath() {
    if (const char* path = std::getenv("RESTAND_POLICY_PATH"); path && *path) return path;
    const std::filesystem::path installed = "/opt/robot/share/motion_master/policy/rl/RL_stand.onnx";
    if (std::filesystem::exists(installed)) return installed;
#ifdef RL_POLICY_SOURCE_DIR
    return std::filesystem::path(RL_POLICY_SOURCE_DIR) / "RL_stand.onnx";
#else
    return installed;
#endif
}

RLStandPolicyRunner::RLStandPolicyRunner(RobotName robot,
                                         const std::filesystem::path& path)
    : PolicyRunnerBase("restand", robot), path_(path) {
    if (robot != RobotName::CR1STANDARD)
        throw std::invalid_argument("restand requires CR1STANDARD / 21 DOF");
    // 参数顺序：左腿6（髋Y/X/Z、膝、踝Y/X）、右腿6、腰Z、左臂4、右臂4。
    kp_ = VecXf::Zero(21);
    kd_ = VecXf::Zero(21);
    kp_ << 300, 300, 200, 300, 50, 30,   // 左腿
           300, 300, 200, 300, 50, 30,   // 右腿
           200,                          // 腰
           100, 100, 100, 100,           // 左臂：肩Y/X/Z、肘
           100, 100, 100, 100;           // 右臂
    kp_ = kp_ * 1.0;
    kd_ << 15, 15, 10, 15, 2, 1,
           15, 15, 10, 15, 2, 1,
           5,
           3, 3, 3, 3,
           3, 3, 3, 3;
    defaults_ = VecXf::Zero(21);
    defaults_ << -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f,
                 -0.2f, 0.0f, 0.0f, 0.4f, -0.2f, 0.0f,
                  0.0f,
                  0.0f, 0.15f, 0.0f, -0.2f,
                  0.0f, -0.15f, 0.0f, -0.2f;

    // SDK 控制周期 1 ms；原仿真周期 5 ms * 4，同为 20 ms 推理一次。
    decimation_ = 20;
    policy_dt_ = 0.020;
    cycle_ = 0.8;
    action_scale_ = 0.25f;
    action_clip_ = 10.0f;
    observation_clip_ = 100.0f;
    omega_scale_ = 1.0f;
    dof_pos_scale_ = 1.0f;
    dof_vel_scale_ = 0.05f;
    command_scale_ = 2.0f;
    options_.SetIntraOpNumThreads(1);
    options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    session_ = Ort::Session(env_, path.c_str(), options_);
    ValidateModel();
    OnEnter();
}

void RLStandPolicyRunner::OnEnter() {
    observation_.setZero();
    history_.setZero();
    action_.setZero();
    phase_time_ = 0;
    run_cnt_ = run_cnt_record_ = 0;
    cmd_vel_input_.setZero();
}

void RLStandPolicyRunner::DisplayPolicyInfo() {
    std::cout << "[restand] " << path_ << ": 330 observations, 13 actions, 50 Hz\n";
}

RobotAction RLStandPolicyRunner::GetRobotAction(const RobotBasicState& state, const UserCommand& user) {
    if (state.joint_pos.size() != 21 || state.joint_vel.size() != 21 ||
        !state.joint_pos.allFinite() || !state.joint_vel.allFinite() ||
        !state.base_rot_mat.allFinite() || !state.base_omega.allFinite() ||
        !std::isfinite(user.forward_vel_scale))
        throw std::runtime_error("restand: invalid feedback or command");
    // STD training allows only forward/backward commands in [-0.5, 0.5].
    const float vx = std::clamp(user.forward_vel_scale, -1.0f, 1.0f) * 0.5f;
    observation_.head(3) = state.base_omega * omega_scale_;
    observation_.segment(3, 3) = state.base_rot_mat.transpose() * Vec3f(0, 0, -1);
    observation_.segment(6, 3) = Vec3f(vx * command_scale_, 0, 0);
    for (int i = 0; i < 21; ++i) {
        observation_(9 + i) = (state.joint_pos(to_robot_[i]) - defaults_(i)) * dof_pos_scale_;
        observation_(30 + i) = state.joint_vel(to_robot_[i]) * dof_vel_scale_;
    }
    observation_.segment(51, 13) = action_;
    const double phase = 2.0 * 3.14159265358979323846 * phase_time_ / cycle_;
    observation_(64) = std::sin(phase);
    observation_(65) = std::cos(phase);
    if (!observation_.allFinite()) throw std::runtime_error("restand: non-finite observation");
    observation_ = observation_.cwiseMax(-observation_clip_).cwiseMin(observation_clip_);
    history_.head(264) = history_.tail(264).eval();
    history_.tail(66) = observation_;
    const std::array<int64_t, 2> shape{{1, 330}};
    auto input = Ort::Value::CreateTensor<float>(memory_, history_.data(), 330, shape.data(), 2);
    auto outputs = session_.Run(Ort::RunOptions{nullptr}, input_names_, &input, 1, output_names_, 1);
    if (outputs[0].GetTensorTypeAndShapeInfo().GetShape() != std::vector<int64_t>({1, 13}))
        throw std::runtime_error("restand: invalid action shape");
    action_ = Eigen::Map<const VecXf>(outputs[0].GetTensorData<float>(), 13);
    if (!action_.allFinite()) throw std::runtime_error("restand: non-finite action");
    action_ = action_.cwiseMax(-action_clip_).cwiseMin(action_clip_);
    RobotAction result;
    result.goal_joint_pos = VecXf::Zero(21);
    result.goal_joint_vel = VecXf::Zero(21);
    result.tau_ff = VecXf::Zero(21);
    result.kp = VecXf::Zero(21);
    result.kd = VecXf::Zero(21);
    for (int i = 0; i < 21; ++i) {
        result.goal_joint_pos(to_robot_[i]) = defaults_(i) + (i < 13 ? action_scale_ * action_(i) : 0);
        result.kp(to_robot_[i]) = kp_(i);
        result.kd(to_robot_[i]) = kd_(i);
    }
    // RestandState 按来源选择姿态，从上一状态最后下发的指令平滑过渡。
    // defaults_ 始终保持训练值；姿态只改变控制目标，不改变观测归一化。
    result.goal_joint_pos(to_robot_[16]) = elbow_target_; // 左肘
    result.goal_joint_pos(to_robot_[20]) = elbow_target_; // 右肘
    // Matches ep_time_buf: stationary/yaw-only commands reset phase time.
    phase_time_ = std::fabs(vx) > 0.1f ? phase_time_ + policy_dt_ : 0;
    ++run_cnt_;
    return result;
}
