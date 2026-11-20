#include "rl_arm_motion_planner.hpp"

namespace pro_policy {

void RLArmMotionPlanner::SetPresetTargets(int preset, const types::Vec7f& left, const types::Vec7f& right) {
    if (preset < 1 || preset > 6 || !left.allFinite() || !right.allFinite())
        throw std::invalid_argument("invalid arm preset targets");
    preset_targets_[preset-1].resize(14);
    preset_targets_[preset-1] << left, right;
}

void RLArmMotionPlanner::Reset(const types::Vec7f& left, const types::Vec7f& right, bool initialized) {
    if (!left.allFinite() || !right.allFinite())
        throw std::invalid_argument("arm initial targets must be finite");
    current_ << left, right;
    start_ = goal_ = udp_output_ = current_;
    previous_presets_.fill(false);
    previous_sequence_ = 0;
    elapsed_ = start_time_ = 0.f;
    initialized_ = initialized;
    interpolating_ = udp_active_ = false;
}

void RLArmMotionPlanner::SetDuration(float duration) {
    if (!std::isfinite(duration) || duration <= .05f)
        throw std::invalid_argument("arm interpolation duration must exceed 0.05 s");
    duration_ = duration;
}

const types::VecXf& RLArmMotionPlanner::Update(const types::RobotBasicState& state,
                          const types::UserCommand& command, float dt, UdpNode* node) {
    elapsed_ += dt;
    const std::array<bool, 6> presets{command.arm_preset_1, command.arm_preset_2,
        command.arm_preset_3, command.arm_preset_4, command.arm_preset_5, command.arm_preset_6};
    if (!initialized_ || command.disable_arm_motion_planning) {
        current_ = state.joint_pos.segment(3, 14);
        initialized_ = true;
    }
    if (command.disable_arm_motion_planning) {
        interpolating_ = false;
    } else {
        const bool sequence_changed = command.arm_preset_seq != previous_sequence_;
        int preset = sequence_changed ? command.arm_preset : 0;
        if (!sequence_changed) {
            for (int i = 0; i < 6; ++i) {
                if (presets[i] && !previous_presets_[i]) { preset = i + 1; break; }
            }
        }
        if (preset >= 1 && preset <= 6) {
            goal_.setZero();
            if (preset == 1 || preset == 5) {
                goal_(1) = .3f; goal_(8) = -.3f;
            } else if (preset == 3 || preset == 6) {
                goal_(1) = .15f; goal_(8) = -.15f;
            }
            if (preset == 6) goal_(3) = goal_(10) = 1.4f;
            if (preset_targets_[preset-1].size() == 14)
                goal_ = preset_targets_[preset-1];
            start_ = current_;
            start_time_ = elapsed_;
            interpolating_ = true;
        }
        if (interpolating_) {
            const float t = std::clamp((elapsed_ - start_time_) / duration_, 0.f, 1.f);
            const float blend = t * t * t * (t * (t * 6.f - 15.f) + 10.f);
            current_ = start_ + blend * (goal_ - start_);
            if (t >= 1.f) { current_ = goal_; interpolating_ = false; }
        }
    }
    previous_presets_ = command.disable_arm_motion_planning ? std::array<bool, 6>{} : presets;
    previous_sequence_ = command.arm_preset_seq;

    if (!node) udp_active_ = false;
    if (node && node->consumeNewTargetData() && !udp_active_) {
        udp_output_ = state.joint_pos.segment(3, 14);
        udp_active_ = true;
    }
    if (udp_active_) {
        static constexpr float lower[14]{
            -3.8397f, -.349066f, -2.9671f, -.767945f, -2.9671f, -1.5708f, -1.5708f,
            -3.8397f, -3.49066f, -2.9671f, -.767945f, -2.9671f, -1.5708f, -1.5708f};
        static constexpr float upper[14]{
            1.5708f, 3.49066f, 2.9671f, 1.74533f, 2.9671f, 1.5708f, 1.5708f,
            1.5708f, .349066f, 2.9671f, 1.74533f, 2.9671f, 1.5708f, 1.5708f};
        for (int i = 0; i < 14; ++i) {
            float target = 0.f;
            node->getTargetPosition(i, target);
            if (!std::isfinite(target)) throw std::runtime_error("non-finite UDP arm target");
            target = std::clamp(target, lower[i], upper[i]);
            const float max_step = (i % 7 < 3 ? 1.5f : 2.f) * dt;
            udp_output_(i) += std::clamp(target - udp_output_(i), -max_step, max_step);
        }
        current_ = udp_output_;
    }
    if (node) {
        for (int i = 0; i < 14; ++i) {
            node->setCurrentPosition(i, state.joint_pos(i + 3));
            node->setCurrentVelocity(i, state.joint_vel(i + 3));
            node->setCurrentTorque(i, state.joint_tau(i + 3));
        }
        node->sendCurrentStates();
    }
    return current_;
}

}  // namespace pro_policy
