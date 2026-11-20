#pragma once

#include "parameters/control_parameters.h"
#include <algorithm>
#include <stdexcept>

namespace humanoid_pro {
// Convert a virtual PD equilibrium into bounded hybrid position/effort fields.
// Both the transmitted feedforward and the total requested effort are bounded;
// the existing driver/checker still checks the resulting packet normally.
class BoundedPolicyCommand {
    types::VecXf lower_, upper_, effort_, velocity_;
public:
    explicit BoundedPolicyCommand(const ControlParameters& cp) {
        if (cp.dof_num_ != 31 || cp.torque_limit_.size() != 31 || cp.joint_vel_limit_.size() != 31)
            throw std::invalid_argument("Bounded policy command requires CR1PRO parameters");
        lower_.resize(31); upper_.resize(31);
        lower_ << cp.waist_joint_lower_, cp.arm_joint_lower_, cp.arm_joint_lower_,
                  cp.leg_joint_lower_, cp.leg_joint_lower_, cp.neck_joint_lower_;
        upper_ << cp.waist_joint_upper_, cp.arm_joint_upper_, cp.arm_joint_upper_,
                  cp.leg_joint_upper_, cp.leg_joint_upper_, cp.neck_joint_upper_;
        // Match the mirrored right roll/yaw limits in JointProtectionChecker.
        for (int i : {11, 12, 24, 25}) {
            const int left = i < 17 ? i - 7 : i - 6;
            lower_(i) = -upper_(left); upper_(i) = -lower_(left);
        }
        effort_ = cp.torque_limit_; velocity_ = cp.joint_vel_limit_;
        if (!lower_.allFinite() || !upper_.allFinite() || !effort_.allFinite() ||
            !velocity_.allFinite() || (lower_.array() > upper_.array()).any() ||
            (effort_.head(29).array() <= 0).any())
            throw std::invalid_argument("Invalid CR1PRO command limits");
    }

    types::MatXf Apply(const types::MatXf& input, const types::RobotBasicState& state) const {
        if (input.rows() != 31 || input.cols() != 5 || !input.allFinite() ||
            state.joint_pos.size() != 31 || state.joint_vel.size() != 31 ||
            !state.joint_pos.allFinite() || !state.joint_vel.allFinite())
            throw std::runtime_error("Invalid hybrid policy command or feedback");
        types::MatXf output = types::MatXf::Zero(31, 5);
        for (int i = 0; i < 29; ++i) {
            const double kp = input(i, 0), kd = input(i, 2);
            if (kp < 0 || kd < 0) throw std::runtime_error("Negative policy gains");
            const double q = state.joint_pos(i), dq = state.joint_vel(i);
            const double limit = effort_(i);
            const double target = std::clamp(double(input(i, 1)), double(lower_(i)), double(upper_(i)));
            const double speed = std::clamp(double(input(i, 3)), -double(velocity_(i)), double(velocity_(i)));
            const double request = kp * (input(i, 1) - q) + kd * (input(i, 3) - dq) + input(i, 4);
            const double bounded_effort = std::clamp(request, -limit, limit);
            const double pd = kp * (target - q) + kd * (speed - dq);
            double scale = 1.;
            // If compensation alone would exceed the effort limit, reduce the
            // PD gains so that bounded feedforward can realize bounded effort.
            if (std::abs(bounded_effort - pd) > limit && std::abs(pd) > 1e-12)
                scale = std::clamp((bounded_effort + std::copysign(limit, pd)) / pd, 0., 1.);
            output(i, 0) = float(kp * scale);
            output(i, 1) = float(target);
            output(i, 2) = float(kd * scale);
            output(i, 3) = float(speed);
            // Calculate using the transmitted float gains to bound roundoff.
            const double actual_pd = output(i, 0) * (target - q) + output(i, 2) * (speed - dq);
            output(i, 4) = float(std::clamp(bounded_effort - actual_pd, -limit, limit));
        }
        return output;
    }
};
}  // namespace humanoid_pro
