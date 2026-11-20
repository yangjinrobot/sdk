#include "rl_stand_salute_policy_runner.hpp"

Vec7f RLStandSalutePolicyRunner::PassThroughVelocity(const Vec7f& incoming, const Vec7f& outgoing) {
    Vec7f velocity = Vec7f::Zero();
    for (int i = 0; i < 7; ++i) {
        const float a = incoming(i), b = outgoing(i);
        if (a*b <= 0.f) continue;  // Stop this joint only when its direction reverses.
        const float harmonic = 2.f*a*b/(a+b);
        // Keep each segment's Bezier control points ordered: no joint overshoot.
        const float limit = 1.2f*std::min(std::abs(a), std::abs(b));
        velocity(i) = std::copysign(std::min(std::abs(harmonic), limit), a);
    }
    return velocity;
}

bool RLStandSalutePolicyRunner::ExternalArmControlEnabled() const { return false; }

RobotAction RLStandSalutePolicyRunner::ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) {
    UserCommand standing = command;
    standing.forward_vel_scale = standing.side_vel_scale = standing.turnning_vel_scale = 0.f;
    standing.disable_arm_motion_planning = true;
    auto result = RLStandPolicyRunner::ComputeRobotAction(state, standing);
    if (!initialized_) {
        left_start_ = state.joint_pos.segment<7>(3);
        right_start_ = state.joint_pos.segment<7>(10);
        initialized_ = true;
    }
    result.goal_joint_pos.segment<7>(3) = left_start_;
    if (left_arm_mode_) {
        const float t = std::clamp(float(left_elapsed_ / kLeftArmDuration), 0.f, 1.f);
        const float blend = t*t*t*(t*(t*6.f-15.f)+10.f);
        result.goal_joint_pos.segment<7>(3) = left_start_ + blend*(LeftArmPose()-left_start_);
        left_elapsed_ += decimation_ * .001;
    }
    if (lowering_requested_.load(std::memory_order_acquire) && !lowering_started_) {
        lowering_start_ = state.joint_pos.segment<7>(10);
        lowering_elapsed_ = 0.;
        lowering_started_ = true;
    }
    if (lowering_started_) {
        const float t = std::clamp(float(lowering_elapsed_ / kLoweringDuration), 0.f, 1.f);
        const float blend = t*t*t*(t*(t*6.f-15.f)+10.f);
        result.goal_joint_pos.segment<7>(10) = lowering_start_ + blend*(RightArmLoweredPose()-lowering_start_);
        // A finished reference alone is insufficient: keep standing until
        // the measured right arm has also reached the lowered posture.
        if (t >= 1.f && (state.joint_pos.segment<7>(10)-RightArmLoweredPose()).cwiseAbs().maxCoeff() <= kLoweredTolerance)
            lowering_complete_.store(true,std::memory_order_release);
        lowering_elapsed_ += decimation_ * .001;
    } else if (!left_arm_mode_) {
        result.goal_joint_pos.segment<7>(10) = RightArmRaisePose(right_start_, elapsed_);
        elapsed_ += decimation_ * .001;
    } else {
        result.goal_joint_pos.segment<7>(10) = right_start_;
    }
    return result;
}

RLStandSalutePolicyRunner::RLStandSalutePolicyRunner(const std::string& name, RobotName robot)
    : RLStandPolicyRunner(name, robot, -0.23f, -0.10f) {}

Vec7f RLStandSalutePolicyRunner::RightArmRaisePose(const Vec7f& start, double elapsed) {
    if (elapsed <= 0.) return start;
    if (elapsed >= kDuration) return RightArmPose();
    const std::array<double, 4> times{0., kChestTime, kLiftTime, kDuration};
    const std::array<Vec7f, 4> poses{start, RightArmChestPose(), RightArmLiftPose(), RightArmPose()};
    std::array<Vec7f, 4> velocity{Vec7f::Zero(), Vec7f::Zero(), Vec7f::Zero(), Vec7f::Zero()};
    for (int i = 1; i < 3; ++i)
        velocity[i] = PassThroughVelocity((poses[i]-poses[i-1])/float(times[i]-times[i-1]),
                                          (poses[i+1]-poses[i])/float(times[i+1]-times[i]));
    const int stage = elapsed < kChestTime ? 0 : elapsed < kLiftTime ? 1 : 2;
    const float duration = float(times[stage+1]-times[stage]);
    const float u = std::clamp(float((elapsed-times[stage])/duration), 0.f, 1.f);
    // Quintic Hermite in Bezier form: shared nonzero knot velocities and zero
    // knot accelerations give C2 continuity without pausing the whole arm.
    std::array<Vec7f, 6> control{
        poses[stage], poses[stage]+duration*velocity[stage]/5.f,
        poses[stage]+2.f*duration*velocity[stage]/5.f,
        poses[stage+1]-2.f*duration*velocity[stage+1]/5.f,
        poses[stage+1]-duration*velocity[stage+1]/5.f, poses[stage+1]};
    for (int count = 5; count > 0; --count)
        for (int i = 0; i < count; ++i)
            control[i] = (1.f-u)*control[i]+u*control[i+1];
    return control[0];
}

Vec7f RLStandSalutePolicyRunner::RightArmChestPose() {
    // Start visible shoulder abduction while folding the elbow toward the chest.
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    return (Vec7f() << -10.f,-40.f,20.f,-20.f,-30.f,0.f,0.f).finished() * degrees_to_radians;
}

Vec7f RLStandSalutePolicyRunner::RightArmLiftPose() {
    // Keep shoulder roll moving outward through both waypoints to the endpoint.
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    return (Vec7f() << -20.f,-75.f,-15.f,-30.f,-65.f,1.f,10.f).finished() * degrees_to_radians;
}

Vec7f RLStandSalutePolicyRunner::RightArmPose() {
    // SDK right arm: shoulder pitch/roll/yaw, elbow, wrist yaw/pitch/roll.
    // Target supplied in degrees; joint commands use radians.
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    return (Vec7f() << -22.f,-100.f,-63.f,-35.f,-84.f,2.f,19.f).finished() * degrees_to_radians;
}

Vec7f RLStandSalutePolicyRunner::RightArmLoweredPose() {
    // Nominal AMP Walking right arm, already down before the state switch.
    return (Vec7f() << 0.f,-.22f,0.f,1.2f,0.f,0.f,0.f).finished();
}

Vec7f RLStandSalutePolicyRunner::LeftArmPose() {
    // SDK left arm: shoulder pitch/roll/yaw, elbow, wrist yaw/pitch/roll.
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    return (Vec7f() << -27.f,11.f,11.f,15.f,-83.f,-2.f,69.f).finished() * degrees_to_radians;
}

void RLStandSalutePolicyRunner::SetLeftArmMotionEnabled(bool enabled) { left_arm_mode_ = enabled; }

void RLStandSalutePolicyRunner::PreservePolicyOnExit(bool preserve) { preserve_policy_on_exit_ = preserve; }

void RLStandSalutePolicyRunner::RequestArmLowering() { lowering_requested_.store(true,std::memory_order_release); }

bool RLStandSalutePolicyRunner::IsArmLoweringComplete() const {
    return lowering_complete_.load(std::memory_order_acquire) && IsReady();
}

void RLStandSalutePolicyRunner::OnEnter() {
    if (!policy_preserved_) {
        RLStandPolicyRunner::OnEnter();
    }
    // Each mode starts its own arm plan at the measured entry pose.
    // Only the standing inference history survives a gesture-mode switch.
    policy_preserved_ = preserve_policy_on_exit_ = false;
    initialized_ = false;
    elapsed_ = 0.;
    left_elapsed_ = 0.;
    lowering_started_ = false;
    lowering_elapsed_ = 0.;
    lowering_requested_.store(false,std::memory_order_release);
    lowering_complete_.store(false,std::memory_order_release);
}

void RLStandSalutePolicyRunner::OnExit() {
    if (preserve_policy_on_exit_) {
        // The next gesture state reuses this runner after its worker joins.
        // Keep ONNX history, while the next mode recaptures both arm poses.
        policy_preserved_ = true;
        preserve_policy_on_exit_ = false;
        return;
    }
    RLStandPolicyRunner::OnExit();
    policy_preserved_ = false;
    initialized_ = false;
    elapsed_ = 0.;
    lowering_started_ = false;
    lowering_elapsed_ = 0.;
    lowering_requested_.store(false,std::memory_order_release);
    lowering_complete_.store(false,std::memory_order_release);
}
