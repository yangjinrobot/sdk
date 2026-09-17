#ifndef CR1_MIMIC_POLICY_RUNNER_H_
#define CR1_MIMIC_POLICY_RUNNER_H_

#include "policy_runner_base.hpp"

#include <array>
#include <filesystem>
#include <string>
#include <vector>

class MimicPolicyRunner : public PolicyRunnerBase {
public:
    static constexpr int kJointCount = 21;
    static constexpr int kObservationSize = 114;

    explicit MimicPolicyRunner(
        RobotName robot_name,
        const std::filesystem::path& policy_directory = DefaultPolicyDirectory());
    MimicPolicyRunner(const std::string& policy_name, RobotName robot_name,
                      const std::filesystem::path& model_path,
                      const std::filesystem::path& reference_path,
                      bool use_readme_deployment_parameters);

    void DisplayPolicyInfo() override;
    void OnEnter() override;
    void OnExit() override;
    RobotAction GetRobotAction(const RobotBasicState& robot, const UserCommand& command) override;

    void AlignHeading(const Mat3f& actual_rotation);
    VecXf InitialJointPosition() const;
    VecXf JointKp() const;
    VecXf JointKd() const;
    bool Finished() const;
    int Frame() const;
    int FrameCount() const;

    static std::filesystem::path DefaultPolicyDirectory();

private:
    struct ReferenceFrame {
        VecXf joint_position;
        VecXf joint_velocity;
        Mat3f anchor_rotation;
    };

    std::string Metadata(const char* key) const;
    VecXf MetadataVector(const char* key) const;
    void ValidateTensor(bool input, const char* name, const std::vector<int64_t>& shape) const;
    void LoadReference(const std::filesystem::path& npz_path);
    VecXf ToRobotOrder(const VecXf& policy_order) const;

    Ort::Env env_{ORT_LOGGING_LEVEL_ERROR, "mofang"};
    Ort::SessionOptions session_options_;
    Ort::Session session_{nullptr};
    Ort::MemoryInfo memory_info_{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};

    std::vector<int> policy_to_robot_;
    VecXf default_joint_position_;
    VecXf kp_;
    VecXf kd_;
    VecXf action_scale_;
    VecXf previous_action_;
    VecXf observation_{VecXf::Zero(kObservationSize)};
    std::vector<ReferenceFrame> reference_;
    Mat3f heading_alignment_{Mat3f::Identity()};
    int frame_{0};
    const char* input_names_[2]{"obs", "time_step"};
};

#endif
