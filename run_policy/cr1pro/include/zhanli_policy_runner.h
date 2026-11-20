#pragma once

#include "policy_runner_base.hpp"
#include "copied_policy_validation.hpp"
#include <array>

// DR02 PRO stand-up policy. This runner is intentionally independent from the
// lie-down runner so each policy's observation and PD metadata are validated
// and mapped without inheriting another policy's configuration.
class ZhanliPolicyRunner : public PolicyRunnerBase {
public:
    explicit ZhanliPolicyRunner(const std::string& policy_name, RobotName robot_name,
                                const std::string& onnx_path = "");
    ~ZhanliPolicyRunner() override = default;

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override;
    bool IsReady() const override { return model_loaded_ && !HasInferenceFault(); }

    int LastFrame() const { return kLastFrame; }

protected:
    RobotAction ComputeRobotAction(const RobotBasicState& state,
                                   const UserCommand& command) override;

private:
    static constexpr int kObservationSize = 151;
    static constexpr int kActionSize = 29;
    static constexpr int kLastFrame = 316;
    static constexpr std::array<int, kActionSize> kPolicyToRobot{
        17, 23, 0, 18, 24, 1, 19, 25, 2, 20, 26, 3, 10, 21, 27,
        4, 11, 22, 28, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16};

    std::vector<Ort::Value> Infer(const VecXf& observation, int frame);
    void CacheReference(const std::vector<Ort::Value>& outputs);
    void LoadModel();

    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "zhanli_policy"};
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    std::string onnx_path_;

    VecXf defaults_ = VecXf::Zero(kActionSize);
    VecXf action_scale_ = VecXf::Zero(kActionSize);
    VecXf kp_ = VecXf::Zero(kActionSize);
    VecXf kd_ = VecXf::Zero(kActionSize);
    VecXf reference_pos_ = VecXf::Zero(kActionSize);
    VecXf reference_vel_ = VecXf::Zero(kActionSize);
    VecXf actor_action_ = VecXf::Zero(kActionSize);
    VecXf last_action_ = VecXf::Zero(kActionSize);
    float clip_actions_ = 10.0f;
    bool model_loaded_ = false;
};
