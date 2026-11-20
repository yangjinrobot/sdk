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
    VecXf dof_pos_default_ = VecXf::Zero(29), kp_ = VecXf::Zero(29), kd_ = VecXf::Zero(29);
    VecXf last_action_ = VecXf::Zero(29), observation_total_ = VecXf::Zero(kInput);
    std::deque<VecXf> obs_history_deque;
    Vec3f max_cmd_vel_{.8f, .8f, 1.57f};
    pro_policy::RLArmMotionPlanner arms_;
    Vec7f initial_left_ = Vec7f::Zero(), initial_right_ = Vec7f::Zero();
    bool initial_arms_pending_ = false;
    inline static UdpNode* udp_ = nullptr;

    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "amp_moe_15dof"};
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_ = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    bool model_loaded_ = false;
    std::string onnx_path_;
    const char* input_names_[1]{"observations"};
    const char* output_names_[1]{"actions"};

    RobotAction ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) override;

public:
    AmpMoe15DofPolicyRunner(const std::string& policy_name, RobotName robot_name);
    static void SetUdpNode(UdpNode* node);
    static UdpNode* GetUdpNode();
    bool IsReady() const override;
    void OnEnter() override;
    void OnExit() override;
    void DisplayPolicyInfo() override;
    void SetDecimation(int value) override;
    void SetCmdMaxVel(const Vec3f& velocity);
    void SetInitialArmTargets(const Vec7f& left, const Vec7f& right);
    void SetArmInterpDuration(float duration);
    const VecXf& GetDefaultJointPosition() const;
};
