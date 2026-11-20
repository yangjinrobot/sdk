#include "rl_stand_talking_policy_runner.hpp"

double RLStandTalkingPolicyRunner::StepDt() const { return decimation_ * .001; }

bool RLStandTalkingPolicyRunner::ExternalArmControlEnabled() const { return false; }

RobotAction RLStandTalkingPolicyRunner::ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) {
    UserCommand standing = command;
    standing.forward_vel_scale = standing.side_vel_scale = standing.turnning_vel_scale = 0.f;
    standing.disable_arm_motion_planning = true;
    auto result = RLStandPolicyRunner::ComputeRobotAction(state, standing);
    if (!initialized_) {
        left_start_ = state.joint_pos.segment<7>(3);
        right_start_ = state.joint_pos.segment<7>(10);
        initialized_ = true;
    }
    // Stage 1 reaches the gesture pose, stage 2 keeps swinging both arms.
    // The shoulder yaw runs in opposition so the arms mirror each other.
    result.goal_joint_pos.segment<7>(3) = GesturePoseAt(left_start_, LeftArmPose(), elapsed_,
                                                        kLeftYawCentreDeg, -1.f);
    result.goal_joint_pos.segment<7>(10) = GesturePoseAt(right_start_, RightArmPose(), elapsed_,
                                                         kRightYawCentreDeg, 1.f);
    if (lowering_requested_.load(std::memory_order_acquire) && !lowering_started_) {
        lowering_start_left_ = state.joint_pos.segment<7>(3);
        lowering_start_right_ = state.joint_pos.segment<7>(10);
        lowering_elapsed_ = 0.;
        lowering_started_ = true;
    }
    if (lowering_started_) {
        const float t = std::clamp(float(lowering_elapsed_ / kLoweringDuration), 0.f, 1.f);
        const float blend = t*t*t*(t*(t*6.f-15.f)+10.f);
        result.goal_joint_pos.segment<7>(3) = lowering_start_left_ + blend*(LeftArmLoweredPose()-lowering_start_left_);
        result.goal_joint_pos.segment<7>(10) = lowering_start_right_ + blend*(RightArmLoweredPose()-lowering_start_right_);
        // A finished reference alone is insufficient: keep standing until
        // both measured arms have also reached the lowered posture.
        if (t >= 1.f &&
            (state.joint_pos.segment<7>(3)-LeftArmLoweredPose()).cwiseAbs().maxCoeff() <= kLoweredTolerance &&
            (state.joint_pos.segment<7>(10)-RightArmLoweredPose()).cwiseAbs().maxCoeff() <= kLoweredTolerance)
            lowering_complete_.store(true,std::memory_order_release);
        lowering_elapsed_ += StepDt();
    }
    elapsed_ += StepDt();
    return result;
}

RLStandTalkingPolicyRunner::RLStandTalkingPolicyRunner(const std::string& name, RobotName robot)
    : RLStandPolicyRunner(name, robot, -0.23f, -0.10f) {}

Vec7f RLStandTalkingPolicyRunner::ArmPoseAt(const Vec7f& start, const Vec7f& target, double elapsed) {
    if (elapsed <= 0.) return start;
    if (elapsed >= kDuration) return target;
    const float t = float(elapsed / kDuration);
    const float blend = t*t*t*(t*(t*6.f-15.f)+10.f);
    return start + blend*(target-start);
}

Vec7f RLStandTalkingPolicyRunner::GesturePoseAt(const Vec7f& start, const Vec7f& target, double elapsed,
                           float yaw_centre_deg, float yaw_sign) {
    Vec7f pose = ArmPoseAt(start, target, elapsed);
    const double swing = elapsed - kDuration;
    if (swing <= 0.) return pose;
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    const float wave = std::sin(float(2. * 3.14159265358979323846 * swing / kSwingPeriod));
    const float ramp = float(std::min(swing / kSwingRamp, 1.));
    const float envelope = ramp*ramp*ramp*(ramp*(ramp*6.f-15.f)+10.f);
    pose(kShoulderYaw) = (1.f-envelope)*pose(kShoulderYaw) +
        envelope*(yaw_centre_deg + yaw_sign*kYawAmplitudeDeg*wave)*degrees_to_radians;
    pose(kElbow) = (1.f-envelope)*pose(kElbow) +
        envelope*(kElbowCentreDeg + kElbowAmplitudeDeg*wave)*degrees_to_radians;
    return pose;
}

Vec7f RLStandTalkingPolicyRunner::RightArmPose() {
    // SDK right arm: shoulder pitch/roll/yaw, elbow, wrist yaw/pitch/roll.
    // Target supplied in degrees; joint commands use radians.
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    return (Vec7f() << -15.f,-10.f,0.f,0.f,0.f,0.f,0.f).finished() * degrees_to_radians;
}

Vec7f RLStandTalkingPolicyRunner::LeftArmPose() {
    // SDK left arm: shoulder pitch/roll/yaw, elbow, wrist yaw/pitch/roll.
    // Target supplied in degrees; joint commands use radians.
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    return (Vec7f() << -15.f,10.f,0.f,0.f,0.f,0.f,0.f).finished() * degrees_to_radians;
}

Vec7f RLStandTalkingPolicyRunner::RightArmLoweredPose() {
    // Nominal AMP Walking right arm, already down before the state switch.
    return (Vec7f() << 0.f,-.22f,0.f,1.2f,0.f,0.f,0.f).finished();
}

Vec7f RLStandTalkingPolicyRunner::LeftArmLoweredPose() {
    // Mirrored nominal AMP Walking left arm, already down before the switch.
    return (Vec7f() << 0.f,.22f,0.f,1.2f,0.f,0.f,0.f).finished();
}

void RLStandTalkingPolicyRunner::PreservePolicyOnExit(bool preserve) { preserve_policy_on_exit_ = preserve; }

void RLStandTalkingPolicyRunner::RequestArmLowering() { lowering_requested_.store(true,std::memory_order_release); }

bool RLStandTalkingPolicyRunner::IsArmLoweringComplete() const {
    return lowering_complete_.load(std::memory_order_acquire) && IsReady();
}

void RLStandTalkingPolicyRunner::OnEnter() {
    if (!policy_preserved_) {
        RLStandPolicyRunner::OnEnter();
    }
    // Each mode starts its own arm plan at the measured entry pose.
    // Only the standing inference history survives a gesture-mode switch.
    policy_preserved_ = preserve_policy_on_exit_ = false;
    initialized_ = false;
    elapsed_ = 0.;
    lowering_started_ = false;
    lowering_elapsed_ = 0.;
    lowering_requested_.store(false,std::memory_order_release);
    lowering_complete_.store(false,std::memory_order_release);
}

void RLStandTalkingPolicyRunner::OnExit() {
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
