#include "amp_walking_policy_runner.hpp"

#include <algorithm>
#include <array>
#include <limits>

static void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Function>
static void RequireThrows(Function function) {
    bool threw = false;
    try { function(); } catch (const std::invalid_argument&) { threw = true; }
    Require(threw, "invalid AMP configuration was accepted");
}

// Independent tensor construction from env(2).yaml / sim2sim_dr02_29dof.py.
// Preserve the current SDK shoulder-roll rest offsets (+/-0.22 rad).
// Indices select SDK joints in the interleaved training order.
static constexpr std::array<int, 29> kPolicyToRobot{
    17, 23, 0, 18, 24, 1, 19, 25, 2, 20, 26, 3, 10, 21, 27,
    4, 11, 22, 28, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16};
static constexpr std::array<float, 29> kDefault{
    -.2f, -.2f, 0, 0, 0, 0, 0, 0, 0, .4f, .4f, 0, 0, -.2f, -.2f,
    .22f, -.22f, 0, 0, 0, 0, 1.2f, 1.2f, 0, 0, 0, 0, 0, 0};
static constexpr std::array<float, 29> kKp{
    300, 300, 200, 300, 300, 200, 200, 200, 300, 300, 300, 100, 100,
    80, 80, 100, 100, 30, 30, 100, 100, 100, 100, 30, 30, 30, 30, 30, 30};
static constexpr std::array<float, 29> kKd{
    10, 10, 5, 10, 10, 5, 5, 5, 10, 10, 10, 5, 5, 3, 3, 5, 5, 1, 1,
    5, 5, 5, 5, 1, 1, 1, 1, 1, 1};

static RobotBasicState State(int step) {
    RobotBasicState state{};
    state.base_rot_mat = RpyToRm(Vec3f(.1f, -.2f, .3f));
    state.base_omega << .02f * step, -.13f, .08f;
    state.joint_pos = VecXf::LinSpaced(31, -.2f, .3f);
    state.joint_vel = VecXf::LinSpaced(31, -.3f, .4f);
    state.joint_tau = VecXf::Zero(31);
    state.joint_pos.array() += .01f * std::sin(.1f * step);
    if (step == 210) state.joint_vel(7) = 3000.f;  // Observation clipping.
    return state;
}

static void CheckRollout(Ort::Session& session, bool legacy_limits) {
    AmpWalkingPolicyRunner runner("AMP 1018 regression", RobotName::CR1PRO);
    Require(runner.IsReady(), runner.GetLastError());
    Require(runner.decimation_ == 20, "AMP must infer at 50 Hz on the 1 kHz SDK loop");
    if (legacy_limits) runner.SetCmdMaxVel(Vec3f(1.25f, 1.f, 1.f));
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const char* inputs[] = {"obs"};
    const char* outputs[] = {"actions"};
    const std::array<int64_t, 2> shape{1, 612};
    // Re-entry must clear history, previous actions, command filter and phase.
    for (int entry = 0; entry < 2; ++entry) {
        runner.OnEnter();
        std::array<float, 510> history{};
        std::array<float, 29> previous{};
        Vec3f filtered = Vec3f::Zero();
        for (int step = 0; step < 360; ++step) {
            const auto state = State(step);
            UserCommand command{};
            // Rest; combined sub-threshold commands; forward/lateral saturation;
            // reverse saturation; stopping; yaw-only command.
            Vec3f requested = Vec3f::Zero();
            if (step >= 10 && step < 40) requested << .07f / 1.2f, .07f / .6f, .07f;
            if (step >= 40 && step < 140) requested << 2.f, -2.f, 0.f;
            if (step >= 140 && step < 260) requested << -2.f, 2.f, -2.f;
            if (step >= 320) requested << 0.f, 0.f, .5f;
            command.forward_vel_scale = requested(0);
            command.side_vel_scale = requested(1);
            command.turnning_vel_scale = requested(2);
            const Vec3f increments(.02f, .02f, .04f);
            for (int i = 0; i < 3; ++i)
                filtered(i) += std::clamp(std::clamp(requested(i), -1.f, 1.f) - filtered(i),
                                          -increments(i), increments(i));
            const Vec3f velocity(std::clamp(filtered(0) * 1.2f, -.5f, 1.2f),
                                 filtered(1) * .6f, filtered(2));
            const Vec3f gravity = state.base_rot_mat.transpose() * Vec3f(0, 0, -1);
            std::array<float, 612> observation{};
            std::copy(history.begin(), history.end(), observation.begin());
            float* current = observation.data() + 510;
            for (int i = 0; i < 3; ++i) {
                current[i] = state.base_omega(i);
                current[3 + i] = gravity(i);
                current[6 + i] = velocity(i);
            }
            for (int i = 0; i < 29; ++i) {
                current[9 + i] = state.joint_pos(kPolicyToRobot[i]) - kDefault[i];
                current[38 + i] = .05f * state.joint_vel(kPolicyToRobot[i]);
                current[67 + i] = previous[i];
            }
            if (velocity.cwiseAbs().maxCoeff() > .1f) {
                // Match float rounding of the reference SDK's 20 x 1 ms clock.
                const float time = static_cast<float>(step) * (20.f * .001f) / .85f;
                for (int leg = 0; leg < 2; ++leg) {
                    const float phase = std::fmod(time + (leg == 0 ? .38f : .88f), 1.f);
                    current[96 + leg] = std::sin(2.f * 3.14159265358979323846f * phase);
                    current[98 + leg] = std::cos(2.f * 3.14159265358979323846f * phase);
                }
            }
            current[100] = current[101] = .38f;
            for (int i = 0; i < 102; ++i) current[i] = std::clamp(current[i], -100.f, 100.f);
            auto tensor = Ort::Value::CreateTensor<float>(memory, observation.data(),
                observation.size(), shape.data(), shape.size());
            auto result = session.Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, 1);
            const auto expected = pro_policy::ReadAction(result, 29);
            const auto actual = runner.GetRobotAction(state, command);
            Require(runner.IsReady(), runner.GetLastError());
            Require(actual.goal_joint_vel.isZero() && actual.tau_ff.isZero(), "unexpected feedforward");
            for (int i = 0; i < 29; ++i) {
                previous[i] = std::clamp(expected(i), -10.f, 10.f);
                const int joint = kPolicyToRobot[i];
                Require(std::abs(actual.goal_joint_pos(joint) -
                        (kDefault[i] + .25f * previous[i])) < 3e-5f,
                        "AMP reference rollout mismatch at step " + std::to_string(step) +
                        ", policy joint " + std::to_string(i));
                Require(actual.kp(joint) == kKp[i] && actual.kd(joint) == kKd[i], "PD gain mapping mismatch");
            }
            // Shift five old frames, then append current after inference.
            std::rotate(history.begin(), history.begin() + 102, history.end());
            std::copy(current, current + 102, history.end() - 102);
        }
        runner.OnExit();
    }
}

int main() {
    try {
        Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "AMP reference");
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
        Ort::Session session(environment,
            pro_policy::ResolveModelPath("amp_policy/amp_1018.onnx").c_str(), options);
        pro_policy::ValidateModelSchema(session, 612, 29, "obs", "actions");
        CheckRollout(session, false);
        CheckRollout(session, true);
        AmpWalkingPolicyRunner runner("AMP configuration", RobotName::CR1PRO);
        RequireThrows([&] { runner.SetCmdMaxVel(Vec3f(-1, 0, 0)); });
        RequireThrows([&] { runner.SetCmdMaxVel(Vec3f(0, std::numeric_limits<float>::infinity(), 0)); });
        RequireThrows([&] { runner.SetCmdVelLimits(Vec3f(-.6f, 0, 0), Vec3f::Zero()); });
        RequireThrows([&] { runner.SetCmdVelLimits(Vec3f::Zero(), Vec3f(1.3f, 0, 0)); });
        RequireThrows([&] { runner.SetCmdVelLimits(Vec3f(.1f, 0, 0), Vec3f::Zero()); });
        RequireThrows([&] { runner.SetCmdVelLimits(Vec3f::Zero(), Vec3f(std::numeric_limits<float>::quiet_NaN(), 0, 0)); });
        RequireThrows([] { AmpWalkingPolicyRunner::GeneratePermutation({"hip"}, {"knee"}); });
        RequireThrows([] { AmpWalkingPolicyRunner::GeneratePermutation({"hip", "hip"}, {"hip"}); });
        runner.SetCmdMaxVel(Vec3f::Zero());
        runner.OnEnter();
        UserCommand command{};
        command.forward_vel_scale = std::numeric_limits<float>::quiet_NaN();
        const auto damping = runner.GetRobotAction(State(0), command);
        Require(!runner.IsReady() && runner.HasInferenceFault() && damping.kp.isZero() &&
                damping.kd.isOnes(), "invalid input did not latch damping");
        runner.OnEnter();
        Require(runner.IsReady(), "re-entry did not clear transient fault");
        std::cout << "AMP 1018: 1440 reference inference comparisons, reset, limits and faults passed\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
