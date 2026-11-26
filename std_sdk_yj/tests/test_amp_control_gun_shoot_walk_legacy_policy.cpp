// From anduidongzuo (assertions must be enabled):
// g++ -O2 -std=c++17 -DEIGEN_DONT_ALIGN_STATICALLY tests/test_amp_control_gun_shoot_walk_legacy_policy.cpp
//   run_policy/cr1/src/{common,chiqiangwalk}_policy_runner.cpp -Irun_policy/cr1/include -Itypes
//   -Iutils -Istate_machine -Ithird_party/eigen -Ithird_party/onnxruntime/x86/include
//   -Lthird_party/onnxruntime/x86/lib -lonnxruntime -pthread -o /tmp/test_chiqiangwalk
// LD_LIBRARY_PATH=third_party/onnxruntime/x86/lib /tmp/test_chiqiangwalk
//   policy/std_amp.onnx
#include "amp_control_gun_shoot_walk_mode_policy_runner.h"
#include "common_policy_runner.h"
#include "arm_entry_transition.hpp"
#include <cassert>
#include <iostream>
#include <limits>

int main(int argc, char** argv) {
    assert(argc == 2);
    AmpControlGunShootWalkModePolicyRunner policy(
        "chiqiangwalk_test", RobotName::CR1STANDARD, argv[1],
        PolicyObservationConfig{78, 5, true});
    assert(policy.obs_total_dim_ == 468 && policy.act_dim_ == 21);
    assert(policy.decimation_ == 20 && policy.action_scale_ == 0.25f);
    assert(policy.kp_(0) == 300 && policy.kd_(0) == 10);
    assert(policy.action_clip_ == 10);

    RobotBasicState state;
    state.base_rot_mat.setIdentity();
    state.base_omega.setZero();
    state.joint_pos = VecXf::Zero(21);
    state.joint_vel = VecXf::Zero(21);
    for (int i = 0; i < 21; ++i)
        state.joint_pos(policy.robot2policy_idx[i]) = policy.dof_pos_default_(i);

    for (int scenario = 0; scenario < 6; ++scenario) {
        UserCommand cmd{};
        if (scenario == 1) cmd.forward_vel_scale = 1;
        if (scenario == 2) cmd.forward_vel_scale = -1;
        if (scenario == 3) cmd.side_vel_scale = 1;
        if (scenario == 4) cmd.turnning_vel_scale = 1;
        if (scenario == 5) {
            cmd.forward_vel_scale = 1;
            cmd.side_vel_scale = -1;
            cmd.turnning_vel_scale = -1;
        }
        policy.OnEnter();
        for (int step = 0; step < 220; ++step) {
            VecXf previous = policy.current_observation_;
            RobotAction action = policy.GetRobotAction(state, cmd);
            assert(action.goal_joint_pos.allFinite());
            assert(policy.action_.allFinite() && policy.action_.cwiseAbs().maxCoeff() <= 10);
            assert(policy.current_observation_(6) >= -0.50001f);
            assert(policy.current_observation_(6) <= 1.20001f);
            assert(std::fabs(policy.current_observation_(7)) <= 0.50001f);
            assert(std::fabs(policy.current_observation_(8)) <= 1.00001f);
            assert(policy.observation_total_.tail(78).isApprox(policy.current_observation_));
            assert(policy.observation_total_.segment(312, 78).isApprox(policy.current_observation_));
            if (step == 0) {
                for (int frame = 0; frame < 6; ++frame)
                    assert(policy.observation_total_.segment(frame * 78, 78).isApprox(policy.current_observation_));
                assert(policy.current_observation_.segment(72, 4).isZero());
                assert(policy.current_observation_.segment(9, 42).isZero());
                Vec3f expected_cmd(.024f * (cmd.forward_vel_scale > 0 ? 1 : cmd.forward_vel_scale < 0 ? -1 : 0),
                                   .01f * cmd.side_vel_scale, .04f * cmd.turnning_vel_scale);
                assert(policy.current_observation_.segment(6, 3).isApprox(expected_cmd, 1e-6f));
            } else {
                assert(policy.observation_total_.segment(234, 78).isApprox(previous));
            }
            if (scenario == 0) assert(policy.current_observation_.segment(6, 3).isZero());
            for (int i = 0; i < 21; ++i) {
                int p = policy.policyandpd2robot_idx[i];
                const float expected = policy.dof_pos_default_(p) + 0.25f * policy.action_(p);
                assert(std::fabs(action.goal_joint_pos(i) - expected) < 1e-5f);
                assert(action.kp(i) == policy.kp_(p) && action.kd(i) == policy.kd_(p));
            }
        }
        if (scenario == 2)
            assert(std::fabs(policy.current_observation_(6) + 0.5f) < 1e-5f);
        policy.OnExit();
    }
    // Safe control still requests zero; release responds without a 2s delay.
    policy.OnEnter();
    UserCommand safety{};
    safety.forward_vel_scale = 1;
    safety.safe_control_mode = 1;
    policy.GetRobotAction(state, safety);
    assert(policy.current_observation_.segment(6, 3).isZero());
    safety.safe_control_mode = 0;
    policy.GetRobotAction(state, safety);
    assert(std::fabs(policy.current_observation_(6) - .024f) < 1e-6f);

    // Check the whole rolling layout using distinct live sensor frames, including
    // re-entry from a different pose. No frame is seeded from the OnEnter pose.
    for (int repeat = 0; repeat < 2; ++repeat) {
        policy.OnEnter();
        assert(policy.observation_history_.isZero() && policy.observation_total_.isZero());
        assert(policy.last_action_.isZero() && policy.cmd_vel_input_.isZero() && policy.run_cnt_ == 0);
        std::deque<VecXf> expected_history;
        for (int step = 0; step < 8; ++step) {
            for (int p = 0; p < 21; ++p) {
                state.joint_pos(policy.robot2policy_idx[p]) = policy.dof_pos_default_(p) + .01f * (p + step + repeat);
                state.joint_vel(policy.robot2policy_idx[p]) = .02f * (p - step);
            }
            state.base_omega = Vec3f(.01f * step, -.03f, .04f);
            const VecXf previous_action = policy.last_action_;
            UserCommand cmd{};
            policy.GetRobotAction(state, cmd);
            assert(policy.current_observation_.segment(51, 21).isApprox(previous_action));
            for (int p = 0; p < 21; ++p) {
                assert(std::fabs(policy.current_observation_(9 + p) - .01f * (p + step + repeat)) < 1e-6f);
                assert(std::fabs(policy.current_observation_(30 + p) - .001f * (p - step)) < 1e-6f);
            }
            if (step == 0) expected_history.assign(5, policy.current_observation_);
            else { expected_history.pop_front(); expected_history.push_back(policy.current_observation_); }
            for (int frame = 0; frame < 5; ++frame)
                assert(policy.observation_total_.segment(frame * 78, 78).isApprox(expected_history[frame]));
            assert(policy.observation_total_.tail(78).isApprox(policy.current_observation_));
        }
        policy.OnExit();
    }
    // Invalid samples must leave history, previous action and inference count intact,
    // both before the first seed and after a successful inference.
    policy.OnEnter();
    auto reject_invalid = [&](const RobotBasicState& bad) {
        const VecXf history = policy.observation_history_, input = policy.observation_total_;
        const VecXf last_action = policy.last_action_;
        const int count = policy.run_cnt_;
        bool rejected = false;
        UserCommand cmd{};
        try { policy.GetRobotAction(bad, cmd); }
        catch (const std::runtime_error&) { rejected = true; }
        assert(rejected);
        assert(policy.observation_history_.isApprox(history));
        assert(policy.observation_total_.isApprox(input));
        assert(policy.last_action_.isApprox(last_action) && policy.run_cnt_ == count);
    };
    for (int valid_frames = 0; valid_frames < 2; ++valid_frames) {
        for (float invalid : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
            RobotBasicState bad = state;
            bad.base_omega(0) = invalid;
            reject_invalid(bad);
            bad = state;
            bad.joint_pos(0) = invalid;
            reject_invalid(bad);
        }
        RobotBasicState bad = state;
        bad.joint_vel.resize(1);
        reject_invalid(bad);
        UserCommand cmd{};
        policy.GetRobotAction(state, cmd);
        if (valid_frames == 0)
            for (int frame = 0; frame < 6; ++frame)
                assert(policy.observation_total_.segment(frame * 78, 78).isApprox(policy.current_observation_));
    }

    // Same nominal current and ONNX, only the historical padding differs.
    state.base_omega.setZero();
    state.joint_vel.setZero();
    for (int p = 0; p < 21; ++p) state.joint_pos(policy.robot2policy_idx[p]) = policy.dof_pos_default_(p);
    policy.OnEnter();
    UserCommand zero{};
    policy.GetRobotAction(state, zero);
    VecXf legacy = VecXf::Zero(468);
    legacy.segment(312, 78) = policy.current_observation_;
    legacy.tail(78) = policy.current_observation_;
    const int64_t shape[] = {1, 468};
    Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "zero_history_test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    Ort::Session session(env, argv[1], options);
    const auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    auto tensor = Ort::Value::CreateTensor<float>(memory, legacy.data(), 468, shape, 2);
    const char* inputs[] = {"obs"};
    const char* outputs[] = {"actions"};
    auto legacy_outputs = session.Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, 1);
    VecXf legacy_action = Eigen::Map<VecXf>(legacy_outputs[0].GetTensorMutableData<float>(), 21);
    legacy_action = legacy_action.cwiseMax(-10).cwiseMin(10);
    std::cout << "Same-current first-frame ankle target difference [degrees, seeded - zero]: "
              << .25f * (policy.action_(15) - legacy_action(15)) * 180.0f / M_PI << ", "
              << .25f * (policy.action_(16) - legacy_action(16)) * 180.0f / M_PI << '\n';
    // Check the legacy runner’s zero-filled initial history.
    CommonPolicyRunner other("unseeded_reference", RobotName::CR1STANDARD, argv[1]);
    other.OnEnter();
    other.GetRobotAction(state, zero);
    assert(other.observation_total_.head(312).isZero());
    assert(other.observation_total_.tail(156).head(78).isApprox(other.current_observation_));
    assert(other.action_.isApprox(legacy_action, 1e-5f));

    // Exercise the same command-rate blend used by AmpControlGunShootWalkModeState, with live
    // ONNX targets updated at 50 Hz and outputs blended at 1 kHz.
    ArmEntryTransition transition;
    const auto start = ArmEntryTransition::Clock::time_point{};
    MatXf entry = MatXf::Zero(8, 5);
    entry.col(0).setConstant(80.f);
    entry.col(1) = state.joint_pos.segment(1, 8).array() + .3f;
    entry.col(2).setConstant(3.f);
    entry.col(3).setConstant(.1f);
    entry.col(4).setConstant(.7f);
    transition.Reset(entry, start);
    policy.OnEnter();
    MatXf target;
    for (int tick = 0; tick <= 1100; ++tick) {
        // Subsequent measurements must not replace the captured entry pose.
        state.joint_pos.segment(1, 8).array() += .00001f;
        if (tick % 20 == 0) {
            UserCommand cmd{};
            target = policy.GetRobotAction(state, cmd).ConvertToMat();
        }
        MatXf output = target;
        transition.Apply(output, 1, start + std::chrono::milliseconds(tick));
        const float s = std::min(1.0f, tick / 1000.0f);
        const float weight = s * s * (3.0f - 2.0f * s);
        for (int i = 0; i < 21; ++i) {
            for (int col = 0; col < 5; ++col) {
                const float expected = i >= 1 && i <= 8 ?
                    (1 - weight) * entry(i - 1, col) + weight * target(i, col) : target(i, col);
                assert(std::fabs(output(i, col) - expected) < 2e-5f);
            }
        }
    }
    // New entry pose after completion; test no switch jump for fixed live targets.
    // Also check CR1PRO's 14 arm joints without affecting waist/legs/neck.
    // Cover arm-only, CR1PRO arm-only and the full CR1STANDARD 21-joint body.
    for (int joints : {8, 14, 21}) {
        const int first = joints == 8 ? 1 : joints == 14 ? 3 : 0;
        entry = MatXf::Constant(joints, 5, .5f);
        transition.Reset(entry, start);
        MatXf fixed = MatXf::Zero(joints == 14 ? 31 : 21, 5);
        fixed.col(1).setConstant(-.2f);
        MatXf begin = fixed, near_end = fixed, end = fixed;
        transition.Apply(begin, first, start);
        assert(begin.block(first, 0, joints, 5).isApprox(entry));
        transition.Apply(near_end, first, start + std::chrono::milliseconds(999));
        transition.Apply(end, first, start + std::chrono::seconds(1));
        assert(end.isApprox(fixed));
        assert((near_end - end).cwiseAbs().maxCoeff() < 3e-6f);
        // Reverse a switch midway: carry the actual last issued command,
        // including PD, velocity and torque, into the opposite transition.
        transition.Reset(entry, start);
        MatXf halfway = fixed;
        transition.Apply(halfway, first, start + std::chrono::milliseconds(500));
        transition.Reset(halfway.block(first, 0, joints, 5), start);
        MatXf reverse_target = fixed;
        reverse_target.block(first, 0, joints, 5) = entry;
        MatXf reverse_begin = reverse_target, reverse_end = reverse_target;
        transition.Apply(reverse_begin, first, start);
        assert(reverse_begin.block(first, 0, joints, 5).isApprox(halfway.block(first, 0, joints, 5)));
        transition.Apply(reverse_end, first, start + std::chrono::seconds(1));
        assert(reverse_end.isApprox(reverse_target));
    }
    std::cout << "PASS: first-frame history seed, rolling layout, invalid samples, re-entry, immediate entry commands and live arm targets\n";
}
