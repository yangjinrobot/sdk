#pragma once

#include "policy_runner_base.hpp"
#include "rl_arm_motion_planner.hpp"
#include <array>
#include <deque>

class AmpMoe15DofPolicyRunner : public PolicyRunnerBase {
    static constexpr int kFrame = 96, kHistory = 5, kInput = 576, kActions = 15;
    static constexpr float kDt = .02f;
    // Observations use the interleaved Isaac/B2 order. The 15-action head has
    // a separate order: waist yaw/roll/pitch, left leg 6, right leg 6.
    // Do not derive the action order by filtering the observation joints.
    inline static constexpr std::array<int, 29> kObservationJoints{
        17, 23, 0, 18, 24, 1, 19, 25, 2, 20, 26, 3, 10, 21, 27,
        4, 11, 22, 28, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16};
    inline static constexpr std::array<int, 15> kActionJoints{
        0, 1, 2, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28};
    VecXf defaults_ = VecXf::Zero(29), kp_ = VecXf::Zero(29), kd_ = VecXf::Zero(29);
    VecXf previous_ = VecXf::Zero(29), input_ = VecXf::Zero(kInput);
    std::deque<VecXf> history_;
    Vec3f max_velocity_{.8f, .8f, 1.57f};
    pro_policy::RLArmMotionPlanner arms_;
    Vec7f initial_left_ = Vec7f::Zero(), initial_right_ = Vec7f::Zero();
    bool initial_arms_pending_ = false;
    inline static UdpNode* udp_ = nullptr;

    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "amp_moe_15dof"};
    Ort::SessionOptions options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    bool loaded_ = false;
    std::string path_;
    const char* inputs_[1]{"observations"};
    const char* outputs_[1]{"actions"};

    RobotAction ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) override {
        Vec3f requested(command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale);
        requested = requested.cwiseMax(-1.f).cwiseMin(1.f);
        const Vec3f increments(.02f, .02f, .04f);
        for (int i = 0; i < 3; ++i)
            cmd_vel_input_(i) += std::clamp(requested(i) - cmd_vel_input_(i), -increments(i), increments(i));

        VecXf frame(kFrame);
        frame.segment<3>(0) = state.base_omega;
        frame.segment<3>(3) = state.base_rot_mat.transpose() * Vec3f(0, 0, -1);
        frame.segment<3>(6) = cmd_vel_input_.cwiseProduct(max_velocity_);
        for (int i = 0; i < 29; ++i) {
            const int joint = kObservationJoints[i];
            frame(9 + i) = state.joint_pos(joint) - defaults_(joint);
            frame(38 + i) = .05f * state.joint_vel(joint);
            frame(67 + i) = previous_(joint);
        }
        pro_policy::ValidateObservation(frame, kFrame);
        frame = frame.cwiseMax(-100.f).cwiseMin(100.f);
        // simtosim.py: repeat the first observation five times; thereafter
        // append current before stacking. The last history frame equals current.
        if (history_.empty()) history_.assign(kHistory, frame);
        else { history_.pop_front(); history_.push_back(frame); }
        for (int i = 0; i < kHistory; ++i) input_.segment(i * kFrame, kFrame) = history_[i];
        input_.tail(kFrame) = frame;
        pro_policy::ValidateObservation(input_, kInput);
        const std::array<int64_t, 2> shape{1, kInput};
        auto tensor = Ort::Value::CreateTensor<float>(memory_, input_.data(), input_.size(), shape.data(), shape.size());
        auto outputs = session_.Run(Ort::RunOptions{nullptr}, inputs_, &tensor, 1, outputs_, 1);
        VecXf action = pro_policy::ReadAction(outputs, kActions);
        action = action.cwiseMax(-100.f).cwiseMin(100.f);

        RobotAction result;
        result.goal_joint_pos = defaults_;
        result.goal_joint_vel = VecXf::Zero(29);
        result.tau_ff = VecXf::Zero(29);
        result.kp = kp_;
        result.kd = kd_;
        previous_.setZero();  // Externally planned arms have zero policy action.
        for (int i = 0; i < kActions; ++i) {
            const int joint = kActionJoints[i];
            previous_(joint) = action(i);
            result.goal_joint_pos(joint) += .25f * action(i);
        }
        result.goal_joint_pos.segment(3, 14) = arms_.Update(state, command, kDt, udp_);
        ++run_cnt_;
        return result;
    }

public:
    AmpMoe15DofPolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {
        decimation_ = 20;
        defaults_ << 0, 0, 0,
            0, .2f, 0, .8f, 0, 0, 0, 0, -.2f, 0, .8f, 0, 0, 0,
            -.15f, 0, 0, .3f, -.15f, 0, -.15f, 0, 0, .3f, -.15f, 0;
        kp_ << 350, 350, 350,
            200, 150, 150, 200, 150, 150, 150, 200, 150, 150, 200, 150, 150, 150,
            350, 350, 200, 350, 100, 100, 350, 350, 200, 350, 100, 100;
        kd_ << 15, 15, 15,
            5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
            10, 10, 5, 10, 5, 5, 10, 10, 5, 10, 5, 5;
        options_.SetIntraOpNumThreads(1);
        options_.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        try {
            path_ = pro_policy::ResolveModelPath("amp_moe_policy/moe_policy_1026.onnx");
            session_ = Ort::Session(env_, path_.c_str(), options_);
            pro_policy::ValidateModelSchema(session_, kInput, kActions, inputs_[0], outputs_[0]);
            loaded_ = true;
            DisplayPolicyInfo();
        } catch (const std::exception& error) { SetInferenceFault(error.what()); }
    }
    static void SetUdpNode(UdpNode* node) { udp_ = node; }
    static UdpNode* GetUdpNode() { return udp_; }
    bool IsReady() const override { return loaded_ && !HasInferenceFault(); }
    void OnEnter() override {
        if (loaded_) ResetInferenceFault();
        previous_.setZero(); input_.setZero(); history_.clear();
        cmd_vel_input_.setZero(); run_cnt_ = 0;
        const Vec7f left = initial_arms_pending_ ? initial_left_ : Vec7f(defaults_.segment<7>(3));
        const Vec7f right = initial_arms_pending_ ? initial_right_ : Vec7f(defaults_.segment<7>(10));
        arms_.Reset(left, right, initial_arms_pending_);
        initial_arms_pending_ = false;
    }
    void OnExit() override {}
    void DisplayPolicyInfo() override {
        std::cout << policy_name_ << ": " << path_ << " observations[1,576] -> actions[1,15], 50 Hz\n";
    }
    void SetDecimation(int value) override {
        if (value != 20) throw std::invalid_argument("AMP MoE requires a 20 ms policy period");
        PolicyRunnerBase::SetDecimation(value);
    }
    void SetCmdMaxVel(const Vec3f& velocity) {
        if (!velocity.allFinite() || (velocity.array() < 0.f).any())
            throw std::invalid_argument("AMP MoE velocity limits must be finite and nonnegative");
        max_velocity_ = velocity.cwiseMin(Vec3f(.8f, .8f, 1.57f));
    }
    void SetInitialArmTargets(const Vec7f& left, const Vec7f& right) {
        if (!left.allFinite() || !right.allFinite()) throw std::invalid_argument("non-finite initial arm targets");
        initial_left_ = left; initial_right_ = right; initial_arms_pending_ = true;
    }
    void SetArmInterpDuration(float duration) { arms_.SetDuration(duration); }
    const VecXf& GetDefaultJointPosition() const { return defaults_; }
};
