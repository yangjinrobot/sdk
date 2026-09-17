#ifndef CR1_SHOOT_DOWN_POLICY_RUNNER_HPP_
#define CR1_SHOOT_DOWN_POLICY_RUNNER_HPP_

#include "policy_runner_base.hpp"

class ShootDownPolicyRunner final : public PolicyRunnerBase {
public:
    explicit ShootDownPolicyRunner(RobotName robot_name);
    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override {}
    RobotAction GetRobotAction(const RobotBasicState& robot, const UserCommand& command) override;
    void AlignHeading(const Mat3f& actual_rotation);
    // The training command clamps to its final reference frame and keeps the
    // actor running.  This reports that the clamp is active; it is not a
    // signal to stop inference.
    bool HoldingFinalFrame() const;
    int Frame() const;
    int FrameCount() const;
    static constexpr int kFrameCount = 267;

private:
    struct ReferenceFrame { VecXf joint_position, joint_velocity; Mat3f anchor_rotation; };
    void LoadReference();
    VecXf ToRobotOrder(const VecXf& policy_order) const;
    std::vector<int> policy_to_robot_{9, 15, 0, 10, 16, 1, 5, 11, 17, 2, 6, 12, 18, 3, 7, 13, 19, 4, 8, 14, 20};
    VecXf default_q_, kp_, kd_, action_scale_, previous_action_, observation_{VecXf::Zero(114)};
    std::vector<ReferenceFrame> reference_;
    Mat3f heading_alignment_{Mat3f::Identity()};
    int frame_{0};
    bool final_frame_was_run_{false};
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "shoot_down"};
    const char* input_names_[2]{"obs", "time_step"};
};

#endif
