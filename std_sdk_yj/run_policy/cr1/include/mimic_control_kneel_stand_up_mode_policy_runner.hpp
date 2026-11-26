#pragma once

#include "policy_runner_base.hpp"
#include <vector>
#include <filesystem>

// This runner loads reference frames directly from the ONNX model.
class MimicControlKneelStandUpModePolicyRunner final : public PolicyRunnerBase {
public:
    static constexpr int kJoints = 21;
    static constexpr int kObs = 5 * kJoints + 9;
    static constexpr int kProjectedGravityObs = 5 * kJoints + 6;
    struct Reference {
        VecXf position, velocity;
        Mat3f anchor_rotation;
    };

private:
    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "mimic_control_kneel_stand_up_mode"};
    Ort::SessionOptions options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
    std::vector<int> policy_to_robot_;
    std::filesystem::path policy_path_;
    VecXf default_, kp_, kd_, scale_, previous_action_, effort_limits_;
    std::vector<Reference> references_;
    VecXf observation_;
    Mat3f heading_alignment_ = Mat3f::Identity();
    int frame_ = 0;
    bool holding_last_frame_ = false;
    const char* input_names_[2] = {"obs", "time_step"};

    std::string Metadata(const char* key) const;

    VecXf MetadataVector(const char* key) const;

    void ValidateTensor(bool input, const char* name, const std::vector<int64_t>& shape) const;

    std::vector<Ort::Value> Infer(const VecXf& observation, int frame, const char* const* outputs, size_t count);

    VecXf ToRobot(const VecXf& policy) const;

public:
    static std::filesystem::path DefaultPolicyDirectory();

    explicit MimicControlKneelStandUpModePolicyRunner(RobotName robot_name = RobotName::CR1STANDARD,
                                 const std::filesystem::path& directory = DefaultPolicyDirectory());

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override {}

    // Real robots cannot teleport to the clip's world heading. Apply only a yaw
    // offset at entry; retain the reference roll/pitch and all subsequent yaw motion.
    void AlignHeading(const Mat3f& actual);
    // sim2sim reset pose, distinct from the fixed training observation/action offset.
    VecXf InitialJointPosition() const { return ToRobot(references_.front().position); }
    VecXf DefaultJointPosition() const { return ToRobot(default_); }
    // Reference pose in SDK joint order; frame is a zero-based motion index.
    VecXf ReferenceJointPosition(int frame) const { return ToRobot(references_.at(frame).position); }
    VecXf JointKp() const { return ToRobot(kp_); }
    VecXf JointKd() const { return ToRobot(kd_); }
    VecXf ActionScale() const { return ToRobot(scale_); }
    const VecXf& EffortLimits() const { return effort_limits_; }
    // Final action has been generated. The FSM must apply it for one full
    // policy period before transitioning; further calls safely clamp to it.
    bool IsMotionFinished() const override { return holding_last_frame_; }
    bool HoldingLastFrame() const { return holding_last_frame_; }
    int Frame() const { return frame_; }
    int FrameCount() const { return static_cast<int>(references_.size()); }
    const VecXf& Observation() const { return observation_; }
    const Reference& ReferenceAt(int frame) const { return references_.at(frame); }

    RobotAction GetRobotAction(const RobotBasicState& robot, const UserCommand&) override;

    // Closed-loop control against frame zero during the AMP -> mimic handoff.
    // Keep updating action history, but do not consume the motion trajectory.
    RobotAction GetPreparationAction(const RobotBasicState& robot);

private:
    void ValidateTraining(const std::filesystem::path& directory);
    RobotAction ComputeAction(const RobotBasicState& robot, bool advance);
};
