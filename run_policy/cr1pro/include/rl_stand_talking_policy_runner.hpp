#pragma once

#include "rl_stand_policy_runner.hpp"
#include <atomic>
#include <cmath>

// Same model, observation layout, history and PD gains as RLStandPolicyRunner.
// Talking is two stage: both arms first reach their gesture pose in kDuration,
// then both shoulder yaw and elbow joints keep swinging on a shared sine.
class RLStandTalkingPolicyRunner : public RLStandPolicyRunner {
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
    // Swing envelope and period shared by every swinging joint.
    static constexpr double kSwingRamp = .5;
    static constexpr double kSwingPeriod = 8.0;
    static constexpr int kShoulderYaw = 2, kElbow = 3;
    // Mirrored shoulder yaw sweep on both arms: left 0..+30 deg and
    // right -30..0 deg, so one arm always holds the negative of the other.
    static constexpr float kYawAmplitudeDeg = 15.f;
    static constexpr float kLeftYawCentreDeg = 15.f, kRightYawCentreDeg = -15.f;
    // Elbow sweeps -15..+10 deg on both arms, in phase.
    static constexpr float kElbowCentreDeg = -2.5f, kElbowAmplitudeDeg = 12.5f;
    RLStandTalkingPolicyRunner(const std::string& name, RobotName robot);

    // Quintic blend with zero velocity and acceleration at both ends: the pose
    // travels from start to target within kDuration without overshoot.
    static Vec7f ArmPoseAt(const Vec7f& start, const Vec7f& target, double elapsed);

    // Stage 2 superposes a shared sine on the gesture pose of one arm. The
    // envelope keeps the velocity continuous at the stage boundary, and every
    // blended sample stays inside the requested joint range. The yaw centre and
    // sign keep the two arms mirrored instead of tracing the same angle.
    static Vec7f GesturePoseAt(const Vec7f& start, const Vec7f& target, double elapsed,
                               float yaw_centre_deg, float yaw_sign);

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
