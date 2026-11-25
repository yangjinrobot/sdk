#pragma once

#include "policy_runner_base.hpp"
#include <algorithm>
#include <array>
#include <deque>

// walktorun/env(15).yaml + policy_1030.onnx: five 96-D frames, no phase or height scan.
// Keep the WAQ runner API/state ID; all 29 joints are controlled by walk-to-run.
class AmpWalkingWAQPolicyRunner : public PolicyRunnerBase {
    static constexpr int kFrame = 96, kHistory = 5, kInput = kFrame * kHistory, kActions = 29;
    // Isaac articulation order -> SDK waist/left arm/right arm/left leg/right leg.
    inline static constexpr std::array<int, kActions> kJoints{
        17, 23, 0, 18, 24, 1, 19, 25, 2, 20, 26, 3, 10, 21, 27,
        4, 11, 22, 28, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16};
    VecXf defaults_ = VecXf::Zero(29), kp_ = VecXf::Zero(29), kd_ = VecXf::Zero(29);
    VecXf previous_ = VecXf::Zero(kActions), input_ = VecXf::Zero(kInput);
    std::deque<VecXf> history_;
    // Deployment limits: cap forward commands below the training maximum of 3 m/s.
    const Vec3f min_velocity_limit_{-.5f, -.5f, -1.57f};
    const Vec3f max_velocity_limit_{2.f, .5f, 1.57f};
    Vec3f min_velocity_ = min_velocity_limit_, max_velocity_ = max_velocity_limit_;

    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "amp_walk_to_run"};
    Ort::SessionOptions options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    bool loaded_ = false;
    std::string path_;
    const char* inputs_[1]{"obs"};
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
        // Match the SDK velocity API while respecting asymmetric deployment limits.
        frame.segment<3>(6) = cmd_vel_input_.cwiseProduct(max_velocity_)
            .cwiseMax(min_velocity_).cwiseMin(max_velocity_);
        for (int i = 0; i < kActions; ++i) {
            const int joint = kJoints[i];
            frame(9 + i) = state.joint_pos(joint) - defaults_(joint);
            frame(38 + i) = .05f * state.joint_vel(joint);
            frame(67 + i) = previous_(i);
        }
        pro_policy::ValidateObservation(frame, kFrame);
        frame = frame.cwiseMax(-100.f).cwiseMin(100.f);
        // Oldest -> newest, including current; fill initial history with the
        // entry observation. This ActorCritic consumes only these five frames.
        if (history_.empty()) history_.assign(kHistory, frame);
        else { history_.pop_front(); history_.push_back(frame); }
        for (int i = 0; i < kHistory; ++i) input_.segment(i * kFrame, kFrame) = history_[i];
        pro_policy::ValidateObservation(input_, kInput);
        const std::array<int64_t, 2> shape{1, kInput};
        auto tensor = Ort::Value::CreateTensor<float>(memory_, input_.data(), input_.size(), shape.data(), shape.size());
        auto outputs = session_.Run(Ort::RunOptions{nullptr}, inputs_, &tensor, 1, outputs_, 1);
        // The environment observes the raw action-buffer entry; clipping is
        // applied separately to position targets and to the observation frame.
        previous_ = pro_policy::ReadAction(outputs, kActions);
        const VecXf action = previous_.cwiseMax(-100.f).cwiseMin(100.f);

        RobotAction result;
        result.goal_joint_pos = defaults_;
        result.goal_joint_vel = VecXf::Zero(29);
        result.tau_ff = VecXf::Zero(29);
        result.kp = kp_;
        result.kd = kd_;
        for (int i = 0; i < kActions; ++i)
            result.goal_joint_pos(kJoints[i]) += .25f * action(i);
        ++run_cnt_;
        return result;
    }

public:
    AmpWalkingWAQPolicyRunner(const std::string& name, RobotName robot) : PolicyRunnerBase(name, robot) {
        decimation_ = 20;
        // Default joint positions and actuator gains from walktorun/env(15).yaml,
        // in SDK order (waist / left arm / right arm / left leg / right leg).
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
            path_ = pro_policy::ResolveModelPath("amp_policy/walktorun/policy_1030.onnx");
            session_ = Ort::Session(env_, path_.c_str(), options_);
            pro_policy::ValidateModelSchema(session_, kInput, kActions, inputs_[0], outputs_[0]);
            loaded_ = true;
        } catch (const std::exception& error) { SetInferenceFault(error.what()); }
    }
    bool IsReady() const override { return loaded_ && !HasInferenceFault(); }
    void OnEnter() override {
        if (loaded_) ResetInferenceFault();
        previous_.setZero(); input_.setZero(); history_.clear();
        cmd_vel_input_.setZero(); run_cnt_ = 0;
    }
    void OnExit() override {}
    void DisplayPolicyInfo() override {
        std::cout << policy_name_ << ": " << path_ << " obs[1,480] -> actions[1,29], 50 Hz\n";
    }
    void SetDecimation(int value) override {
        if (value != 20) throw std::invalid_argument("AMP Walking WAQ requires a 20 ms policy period");
        PolicyRunnerBase::SetDecimation(value);
    }
    void SetCmdMaxVel(const Vec3f& velocity) {
        if (!velocity.allFinite() || (velocity.array() < 0.f).any())
            throw std::invalid_argument("AMP Walking WAQ velocity limits must be finite and nonnegative");
        max_velocity_ = velocity.cwiseMin(max_velocity_limit_);
        min_velocity_ = (-max_velocity_).cwiseMax(min_velocity_limit_);
    }
    const VecXf& GetDefaultJointPosition() const { return defaults_; }
};
