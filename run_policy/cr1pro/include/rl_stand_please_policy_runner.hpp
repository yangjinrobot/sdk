#pragma once

#include "rl_stand_policy_runner.hpp"
#include <atomic>
#include <cmath>

// Same model, observation layout, history and PD gains as RLStandPolicyRunner.
// Please specializes the hip/ankle defaults, velocity commands and the targets
// of both arms, which follow one synchronized timeline to a single pose.
class RLStandPleasePolicyRunner : public RLStandPolicyRunner {
    Vec7f left_start_ = Vec7f::Zero(), right_start_ = Vec7f::Zero();
    bool initialized_ = false;
    double elapsed_ = 0.;
    Vec7f lowering_start_left_ = Vec7f::Zero(), lowering_start_right_ = Vec7f::Zero();
    bool lowering_started_ = false;
    double lowering_elapsed_ = 0.;
    std::atomic<bool> lowering_requested_{false}, lowering_complete_{false};
    bool preserve_policy_on_exit_ = false, policy_preserved_ = false;

    // signal_dt_ is private in the base runner; the policy runs at 1kHz.
    double StepDt() const;

protected:
    bool ExternalArmControlEnabled() const override;

    RobotAction ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) override;

public:
    static constexpr double kDuration = 3.0;
    static constexpr double kLoweringDuration = 2.4;
    static constexpr float kLoweredTolerance = .15f;
    RLStandPleasePolicyRunner(const std::string& name, RobotName robot);

    // Quintic blend with zero velocity and acceleration at both ends: the pose
    // travels from start to target within kDuration without overshoot.
    static Vec7f ArmPoseAt(const Vec7f& start, const Vec7f& target, double elapsed);

    static Vec7f RightArmPose();
    static Vec7f LeftArmPose();
    static Vec7f RightArmLoweredPose();
    static Vec7f LeftArmLoweredPose();
    // Only the controller lifecycle reads/writes this flag, never inference.
    void PreservePolicyOnExit(bool preserve);
    void RequestArmLowering();
    bool IsArmLoweringComplete() const;
    void OnEnter() override;
    void OnExit() override;
};
