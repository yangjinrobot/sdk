// Build with the same include/library flags as test_chiqiangwalk_policy.cpp.
// Sources: this file and run_policy/cr1/src/run_policy_runner.cpp.
// argv[1]: policy/amp_dt/amp_dt.onnx.
#include "run_policy_runner.h"
#include <cassert>
#include <limits>
#include <fstream>
#include <iomanip>

int main(int argc, char** argv) {
    assert(argc == 2 || argc == 3);
    std::ofstream trace;
    if (argc == 3) { trace.open(argv[2]); trace << std::setprecision(9); }
    RunPolicyRunner policy("run_test", RobotName::CR1STANDARD, argv[1]);
    assert(policy.obs_total_dim_ == 468 && policy.act_dim_ == 21 && policy.decimation_ == 20);
    assert(policy.dof_pos_default_(17) == -0.2f && policy.dof_pos_default_(18) == -0.2f);
    assert(policy.action_clip_ == 10 && policy.observation_clip_ == 100);
    assert(policy.phase_active_at_threshold_);
    if (trace) {
        for (const auto& item : std::vector<std::pair<std::string, VecXf>>{
                {"defaults", policy.dof_pos_default_}, {"kp", policy.kp_}, {"kd", policy.kd_},
                {"max_cmd", policy.max_cmd_vel_}}) {
            trace << item.first << " " << item.second.transpose() << "\n";
        }
        trace << "scales " << policy.action_scale_ << " " << policy.action_clip_ << " "
              << policy.observation_clip_ << " " << policy.omega_scale_ << " "
              << policy.dof_vel_scale_ << "\n";
    }
    RobotBasicState state;
    state.base_rot_mat.setIdentity();
    state.base_omega.setZero();
    state.joint_pos = VecXf::Zero(21);
    state.joint_vel = VecXf::Zero(21);
    for (int episode = 0; episode < 2; ++episode) {
        policy.OnEnter();
        assert(policy.observation_history_.isZero() && policy.last_action_.isZero());
        std::deque<VecXf> expected;
        for (int step = 0; step <= 120; ++step) {
            for (int i = 0; i < 21; ++i)
                state.joint_pos(policy.robot2policy_idx[i]) = policy.dof_pos_default_(i) +
                    .001f * (episode + step % 5);
            if (step == 0 || step == 3) {
                const VecXf history = policy.observation_history_;
                const int count = policy.run_cnt_;
                for (float invalid : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
                    RobotBasicState bad = state;
                    bad.joint_pos(0) = invalid;
                    bool rejected = false;
                    try { policy.GetRobotAction(bad, UserCommand{}); }
                    catch (const std::runtime_error&) { rejected = true; }
                    assert(rejected && policy.run_cnt_ == count);
                    assert(policy.observation_history_.isApprox(history));
                }
            }
            UserCommand cmd{};
            cmd.forward_vel_scale = 1;
            const auto result = policy.GetRobotAction(state, cmd);
            if (trace) trace << "sample " << policy.observation_total_.transpose() << " "
                             << policy.action_.transpose() << "\n";
            assert(result.goal_joint_pos.allFinite());
            if (step == 0) expected.assign(5, policy.current_observation_);
            else { expected.pop_front(); expected.push_back(policy.current_observation_); }
            for (int frame = 0; frame < 5; ++frame)
                assert(policy.observation_total_.segment(frame * 78, 78).isApprox(expected[frame]));
            assert(policy.observation_total_.tail(78).isApprox(policy.current_observation_));
            assert(std::fabs(policy.current_observation_(6) - std::min(1.5f, .03f * (step + 1))) < 3e-6f);
            for (int i = 0; i < 21; ++i) {
                const int p = policy.policyandpd2robot_idx[i];
                assert(std::fabs(result.goal_joint_pos(i) - policy.dof_pos_default_(p) - .25f * policy.action_(p)) < 1e-6f);
                assert(result.kp(i) == policy.kp_(p) && result.kd(i) == policy.kd_(p));
            }
        }
        policy.OnExit();
    }
    // Physical command boundaries on every axis, after the command ramp.
    for (const Vec3f physical : std::vector<Vec3f>{
            Vec3f::Zero(), Vec3f(.099f, 0, 0), Vec3f(.1f, 0, 0),
            Vec3f(-.099f, 0, 0), Vec3f(-.1f, 0, 0),
            Vec3f(0, .099f, 0), Vec3f(0, .1f, 0),
            Vec3f(0, 0, .099f), Vec3f(0, 0, .1f)}) {
        policy.OnEnter();
        policy.run_cnt_ = 100;
        policy.cmd_vel_input_ = physical.cwiseQuotient(policy.max_cmd_vel_);
        UserCommand cmd{};
        cmd.forward_vel_scale = physical(0) / (physical(0) < 0 ? .5f : 1.5f);
        cmd.side_vel_scale = physical(1) / .6f;
        cmd.turnning_vel_scale = physical(2);
        policy.GetRobotAction(state, cmd);
        assert(policy.current_observation_.segment(6, 3).isApprox(physical, 1e-6f));
        if (physical.cwiseAbs().maxCoeff() < .1f)
            assert(policy.current_observation_.segment(72, 4).isZero());
        else {
            for (int leg = 0; leg < 2; ++leg) {
                const float s = policy.current_observation_(72 + leg);
                const float c = policy.current_observation_(74 + leg);
                assert(std::fabs(s*s + c*c - 1) < 1e-5f);
            }
        }
    }
    // Saturation must respect each YAML range, including asymmetric reverse X.
    for (int axis = 0; axis < 3; ++axis) for (float sign : {-1.f, 1.f}) {
        policy.OnEnter();
        UserCommand cmd{};
        if (axis == 0) cmd.forward_vel_scale = sign * 3;
        if (axis == 1) cmd.side_vel_scale = sign * 3;
        if (axis == 2) cmd.turnning_vel_scale = sign * 3;
        for (int step = 0; step < 220; ++step) policy.GetRobotAction(state, cmd);
        const float expected = axis == 0 ? (sign < 0 ? -.5f : 1.5f) :
                               sign * policy.max_cmd_vel_(axis);
        assert(std::fabs(policy.current_observation_(6 + axis) - expected) < 1e-5f);
    }
    policy.OnEnter();
    const int count = policy.run_cnt_;
    UserCommand invalid_command{};
    invalid_command.forward_vel_scale = std::numeric_limits<float>::quiet_NaN();
    bool rejected = false;
    try { policy.GetRobotAction(state, invalid_command); }
    catch (const std::runtime_error&) { rejected = true; }
    assert(rejected && policy.run_cnt_ == count && policy.observation_history_.isZero());
    state.joint_pos.setConstant(1000.f);
    policy.GetRobotAction(state, UserCommand{});
    assert(policy.current_observation_.cwiseAbs().maxCoeff() <= 100);
    std::cout << "PASS: amp_dt parameters, phase boundary, velocity limits, observation clipping;  nirenwalk first-frame seed, rolling history, re-entry, invalid samples, joint mapping and immediate entry commands\n";
}
