// Offline runner integration: compile as test_nirenwalk_policy.cpp; pass a
// 73-D run ONNX model (for example policy/1028_run/policy_run_1.onnx) as argv[1].
#include "run_policy_runner.h"
#include <cassert>
#include <iostream>
#include <utility>

namespace {
constexpr double dt = .02;

void CheckCurve(double start, double target) {
    RunDeceleration filter;
    double velocity = start, previous_acceleration = 0.0;
    int settled = -1;
    for (int tick = 0; tick < 400; ++tick) {
        const double next = filter.Update(velocity, target, target, dt);
        const double acceleration = (next - velocity) / dt;
        assert(std::abs(acceleration) <= 1.000001);
        assert(std::abs(acceleration - previous_acceleration) <= 3.0 * dt + 1e-6);
        assert(next >= std::min(start, target) - 1e-7);
        assert(next <= std::max(start, target) + 1e-7);
        assert((next - velocity) * (target - start) >= -1e-7);
        velocity = next;
        previous_acceleration = acceleration;
        if (!filter.active()) { settled = tick; break; }
    }
    assert(settled > 0 && velocity == target);
    std::cout << "curve " << start << " -> " << target << ": " << (settled + 1) * dt << " s\n";
}
}

int main(int argc, char** argv) {
    assert(argc == 2);
    for (const auto& pair : {std::pair<double, double>{2., 0.}, {1.2, .5},
                             {1.01, 0.}, {2., -1.}, {-1.5, 0.}})
        CheckCurve(pair.first, pair.second);
    // Rapidly changing targets must not reset acceleration on each tick.
    RunDeceleration filter;
    double velocity = 2., previous_acceleration = 0.;
    for (int tick = 0; tick < 600; ++tick) {
        const double target = tick < 40 ? 0. : tick < 80 ? 1.8 :
                              tick < 160 ? .4 + .3 * std::sin(tick * .31) : 0.;
        const double next = filter.Update(velocity, target, target, dt);
        const double acceleration = (next - velocity) / dt;
        assert(std::abs(acceleration) <= 1.000001);
        assert(std::abs(acceleration - previous_acceleration) <= 3. * dt + 1e-5);
        assert(next >= -1.000001 && next <= 2.000001);
        velocity = next;
        previous_acceleration = acceleration;
    }
    assert(std::abs(velocity) < 1e-7 && !filter.active());
    filter.Update(2., 0., 0., dt);
    filter.Reset();
    assert(!filter.active() && filter.acceleration() == 0.);
    assert(filter.Update(1., 0., .96, dt) == .96); // strict >1 threshold

    RunPolicyRunner policy(
        "deceleration_test", RobotName::CR1STANDARD, argv[1],
        RunPolicyRunner::Variant::Run1027);
    RobotBasicState state;
    state.base_rot_mat.setIdentity();
    state.base_omega.setZero();
    state.joint_pos = VecXf::Zero(21);
    state.joint_vel = VecXf::Zero(21);
    UserCommand command{};
    policy.OnEnter();
    command.forward_vel_scale = 1.f;
    for (int tick = 0; tick < 60; ++tick) policy.GetRobotAction(state, command);
    assert(std::abs(policy.current_observation_(6) - 2.f) < 1e-6f);
    velocity = 2.; previous_acceleration = 0.;
    command.forward_vel_scale = 0.f;
    command.side_vel_scale = 1.f;
    command.turnning_vel_scale = -1.f;
    for (int tick = 0; tick < 200; ++tick) {
        policy.GetRobotAction(state, command);
        const float shaped_vx = policy.cmd_vel_input_(0) * policy.max_cmd_vel_(0);
        const double next = shaped_vx;
        assert(policy.current_observation_(6) == (std::abs(shaped_vx) < .1f ? 0.f : shaped_vx));
        const double acceleration = (next - velocity) / dt;
        assert(std::abs(acceleration) <= 1.0001);
        assert(std::abs(acceleration - previous_acceleration) <= 3. * dt + .001);
        assert(next <= velocity + 1e-6 && next >= -1e-6);
        if (tick == 0) {
            assert(next > 1.998 && next < 2.);
            assert(policy.current_observation_(7) == 0.f);
            assert(policy.current_observation_(8) == 0.f);
        }
        velocity = next;
        previous_acceleration = acceleration;
    }
    assert(std::abs(velocity) < 1e-7);
    assert(std::abs(policy.current_observation_(7) - .6f) < 1e-6);
    assert(std::abs(policy.current_observation_(8) + 1.f) < 1e-6);
    // Low-speed ramp and re-entry retain legacy behavior.
    policy.OnExit(); policy.OnEnter();
    command = UserCommand{};
    command.forward_vel_scale = .4f;
    policy.GetRobotAction(state, command);
    assert(std::abs(policy.cmd_vel_input_(0) * policy.max_cmd_vel_(0) - .04f) < 1e-6);
    assert(policy.current_observation_(6) == 0.f);
    for (int tick = 0; tick < 30; ++tick) policy.GetRobotAction(state, command);
    command.forward_vel_scale = 0.f;
    policy.GetRobotAction(state, command);
    assert(std::abs(policy.current_observation_(6) - .76f) < 1e-6);
    // Direct runner safety request bypasses S shaping and keeps legacy behavior.
    command.forward_vel_scale = 1.f;
    for (int tick = 0; tick < 60; ++tick) policy.GetRobotAction(state, command);
    command.forward_vel_scale = 0.f;
    policy.GetRobotAction(state, command);
    command.safe_control_mode = 1;
    const float before = policy.current_observation_(6);
    policy.GetRobotAction(state, command);
    assert(std::abs(policy.current_observation_(6) - (before - .04f)) < 1e-6);
    policy.OnEnter();
    command = UserCommand{};
    policy.GetRobotAction(state, command);
    assert(policy.current_observation_.segment(6, 3).isZero());
    std::cout << "PASS: S curves, retargeting, jerk/deceleration limits, physical command observations, low speed, re-entry and safety bypass\n";
}
