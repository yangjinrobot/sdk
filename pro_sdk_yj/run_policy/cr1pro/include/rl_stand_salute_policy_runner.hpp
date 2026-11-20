#pragma once

#include "rl_stand_policy_runner.hpp"
#include <array>
#include <atomic>
#include <cmath>

// Same model, observation layout, history and PD gains as RLStandPolicyRunner.
// Salute specializes the hip/ankle defaults, velocity commands and arm targets.
class RLStandSalutePolicyRunner : public RLStandPolicyRunner {
    Vec7f left_start_ = Vec7f::Zero(), right_start_ = Vec7f::Zero();
    bool initialized_ = false;
    double elapsed_ = 0.;
    Vec7f lowering_start_ = Vec7f::Zero();
    bool lowering_started_ = false;
    double lowering_elapsed_ = 0.;
    std::atomic<bool> lowering_requested_{false}, lowering_complete_{false};
    bool left_arm_mode_ = false;
    bool preserve_policy_on_exit_ = false, policy_preserved_ = false;
    double left_elapsed_ = 0.;

    static Vec7f PassThroughVelocity(const Vec7f& incoming, const Vec7f& outgoing);

protected:
    bool ExternalArmControlEnabled() const override;

    RobotAction ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) override;

public:
    static constexpr double kChestTime = 1.2;
    static constexpr double kLiftTime = 2.2;
    static constexpr double kDuration = 3.0;
    static constexpr double kLoweringDuration = 2.4;
    static constexpr double kLeftArmDuration = 3.0;
    static constexpr float kLoweredTolerance = .15f;
    RLStandSalutePolicyRunner(const std::string& name, RobotName robot);

    static Vec7f RightArmRaisePose(const Vec7f& start, double elapsed);

    static Vec7f RightArmChestPose();
    static Vec7f RightArmLiftPose();
    static Vec7f RightArmPose();
    static Vec7f RightArmLoweredPose();
    static Vec7f LeftArmPose();
    // Called only while the shared policy worker is stopped between states.
    void SetLeftArmMotionEnabled(bool enabled);
    // Only the controller lifecycle reads/writes this flag, never inference.
    void PreservePolicyOnExit(bool preserve);
    void RequestArmLowering();
    bool IsArmLoweringComplete() const;
    void OnEnter() override;
    void OnExit() override;
};
