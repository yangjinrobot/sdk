#pragma once

#include "policy_runner_base.hpp"
#include "copied_policy_validation.hpp"
#include "rl_arm_motion_planner.hpp"
#include <array>

// Knee mimic exports: 151 observations, projected gravity, 50 Hz and a
// stationary terminal reference. Normalization is already inside the ONNX.
class KneeMimicPolicyRunner : public PolicyRunnerBase {
public:
    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override;
    bool IsReady() const override { return model_loaded_ && !HasInferenceFault(); }

protected:
    KneeMimicPolicyRunner(const std::string& name, RobotName robot,
                          const std::string& relative_path, int last_frame,
                          const std::string& override_path,
                          bool independent_terminal_arms = false, UdpNode* udp = nullptr);
    RobotAction ComputeRobotAction(const RobotBasicState&, const UserCommand&) override;

private:
    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "knee_mimic"};
    Ort::SessionOptions options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    std::string path_;
    const int last_frame_;
    bool model_loaded_ = false;
    VecXf kp_, kd_, defaults_, scale_;
    VecXf reference_pos_ = VecXf::Zero(29), reference_vel_ = VecXf::Zero(29);
    VecXf last_action_ = VecXf::Zero(29);
    const bool independent_terminal_arms_;
    bool terminal_arms_started_ = false;
    pro_policy::RLArmMotionPlanner arms_;
    UdpNode* udp_ = nullptr;  // Owned by the state machine; no sockets started here.
    // Isaac articulation order -> SDK waist/arms/legs order.
    inline static constexpr std::array<int, 29> policy_to_robot_{
        17,23,0,18,24,1,19,25,2,20,26,3,10,21,27,4,11,22,28,5,12,6,13,7,14,8,15,9,16};
    std::vector<Ort::Value> Infer(VecXf& observation, int frame);
    void CacheReference(const std::vector<Ort::Value>& output);
};

class Stand2KneePolicyRunner : public KneeMimicPolicyRunner {
public:
    Stand2KneePolicyRunner(const std::string& name, RobotName robot,
                          const std::string& path = "", UdpNode* udp = nullptr)
        : KneeMimicPolicyRunner(name, robot,
              "mimic_policy/stand2knee/model_stand2knee_1028.onnx", 383, path, true, udp) {}
};

class Knee2StandPolicyRunner : public KneeMimicPolicyRunner {
public:
    Knee2StandPolicyRunner(const std::string& name, RobotName robot, const std::string& path = "")
        : KneeMimicPolicyRunner(name, robot,
              "mimic_policy/knee2stand/model_knee2stand_1028.onnx", 312, path) {}
};
