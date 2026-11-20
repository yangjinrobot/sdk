#include "rl_stand_please_policy_runner.hpp"

double RLStandPleasePolicyRunner::StepDt() const { return decimation_ * .001; }

bool RLStandPleasePolicyRunner::ExternalArmControlEnabled() const { return false; }

RobotAction RLStandPleasePolicyRunner::ComputeRobotAction(const RobotBasicState& state, const UserCommand& command) {
    UserCommand standing = command;
    standing.forward_vel_scale = standing.side_vel_scale = standing.turnning_vel_scale = 0.f;
    standing.disable_arm_motion_planning = true;
    auto result = RLStandPolicyRunner::ComputeRobotAction(state, standing);
    if (!initialized_) {
        left_start_ = state.joint_pos.segment<7>(3);
        right_start_ = state.joint_pos.segment<7>(10);
        initialized_ = true;
    }
    // Both arms advance on the same clock towards their single target pose.
    result.goal_joint_pos.segment<7>(3) = ArmPoseAt(left_start_, LeftArmPose(), elapsed_);
    result.goal_joint_pos.segment<7>(10) = ArmPoseAt(right_start_, RightArmPose(), elapsed_);
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

RLStandPleasePolicyRunner::RLStandPleasePolicyRunner(const std::string& name, RobotName robot)
    : RLStandPolicyRunner(name, robot, -0.23f, -0.10f) {}

Vec7f RLStandPleasePolicyRunner::ArmPoseAt(const Vec7f& start, const Vec7f& target, double elapsed) {
    if (elapsed <= 0.) return start;
    if (elapsed >= kDuration) return target;
    const float t = float(elapsed / kDuration);
    const float blend = t*t*t*(t*(t*6.f-15.f)+10.f);
    return start + blend*(target-start);
}

Vec7f RLStandPleasePolicyRunner::RightArmPose() {
    // SDK right arm: shoulder pitch/roll/yaw, elbow, wrist yaw/pitch/roll.
    // Target supplied in degrees; joint commands use radians.
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    return (Vec7f() << -18.f,-10.f,37.f,9.f,2.f,-10.f,0.f).finished() * degrees_to_radians;
}

Vec7f RLStandPleasePolicyRunner::LeftArmPose() {
    // SDK left arm: shoulder pitch/roll/yaw, elbow, wrist yaw/pitch/roll.
    // Target supplied in degrees; joint commands use radians.
    constexpr float degrees_to_radians = 3.14159265358979323846f / 180.f;
    return (Vec7f() << -27.f,24.f,36.f,62.f,0.f,6.f,16.f).finished() * degrees_to_radians;
}

Vec7f RLStandPleasePolicyRunner::RightArmLoweredPose() {
    // Nominal AMP Walking right arm, already down before the state switch.
    return (Vec7f() << 0.f,-.22f,0.f,1.2f,0.f,0.f,0.f).finished();
}

Vec7f RLStandPleasePolicyRunner::LeftArmLoweredPose() {
    // Mirrored nominal AMP Walking left arm, already down before the switch.
    return (Vec7f() << 0.f,.22f,0.f,1.2f,0.f,0.f,0.f).finished();
}

void RLStandPleasePolicyRunner::PreservePolicyOnExit(bool preserve) { preserve_policy_on_exit_ = preserve; }

void RLStandPleasePolicyRunner::RequestArmLowering() { lowering_requested_.store(true,std::memory_order_release); }

bool RLStandPleasePolicyRunner::IsArmLoweringComplete() const {
    return lowering_complete_.load(std::memory_order_acquire) && IsReady();
}

void RLStandPleasePolicyRunner::OnEnter() {
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

void RLStandPleasePolicyRunner::OnExit() {
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
