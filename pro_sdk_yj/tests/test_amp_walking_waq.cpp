#include "amp_walking_waq_policy_runner.hpp"
#include <array>

static void Require(bool ok, const std::string& message) {
    if (!ok) throw std::runtime_error(message);
}

static RobotBasicState Feedback(int step) {
    RobotBasicState state{};
    state.base_rot_mat = RpyToRm(Vec3f(.08f, -.05f, .12f));
    state.base_omega << .01f * std::sin(.2f * step), -.03f, .02f;
    state.joint_pos = VecXf::LinSpaced(31, -.03f, .04f);
    state.joint_pos.array() += .005f * std::sin(.1f * step);
    state.joint_vel = VecXf::LinSpaced(31, -.1f, .15f);
    state.joint_tau = VecXf::Zero(31);
    if (step == 80) state.joint_vel(5) = 2500.f;
    return state;
}

static void CheckPolicy() {
    // Defaults and PD from walktorun/env(15).yaml in Isaac articulation order;
    // compare all 29 outputs, including arms, against the new ONNX directly.
    const std::array<int, 29> joints{17,23,0,18,24,1,19,25,2,20,26,3,10,21,27,
        4,11,22,28,5,12,6,13,7,14,8,15,9,16};
    const std::array<float, 29> defaults{-.15f,-.15f,0,0,0,0,0,0,0,.3f,.3f,0,0,
        -.15f,-.15f,.2f,-.2f,0,0,0,0,.8f,.8f,0,0,0,0,0,0};
    const std::array<float, 29> kp{350,350,350,350,350,350,200,200,350,350,350,
        200,200,100,100,150,150,100,100,150,150,200,200,150,150,150,150,150,150};
    const std::array<float, 29> kd{10,10,15,10,10,15,5,5,15,10,10,
        5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
    Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "WAQ-reference");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    Ort::Session session(env, pro_policy::ResolveModelPath("amp_policy/walktorun/policy_1030.onnx").c_str(), options);
    pro_policy::ValidateModelSchema(session, 480, 29, "obs", "actions");
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const char* inputs[]{"obs"}; const char* outputs[]{"actions"};
    const std::array<int64_t, 2> shape{1,480};
    AmpWalkingWAQPolicyRunner runner("waq-regression", RobotName::CR1PRO);
    Require(runner.IsReady(), runner.GetLastError());
    Require(runner.decimation_ == 20, "WAQ policy period mismatch");
    for (int entry = 0; entry < 3; ++entry) {
        if (entry == 1) runner.SetCmdMaxVel(Vec3f(6.f, 2.f, 2.f));
        if (entry == 2) runner.SetCmdMaxVel(Vec3f(.4f, .3f, .7f));
        const Vec3f limits = entry == 2 ? Vec3f(.4f,.3f,.7f) : Vec3f(2.f,.5f,1.57f);
        const Vec3f minimum = entry == 2 ? Vec3f(-.4f,-.3f,-.7f) : Vec3f(-.5f,-.5f,-1.57f);
        runner.OnEnter();
        std::deque<std::array<float,96>> history;
        std::array<float,29> previous{};
        Vec3f filtered = Vec3f::Zero();
        for (int step = 0; step < 240; ++step) {
            const auto state = Feedback(step);
            UserCommand command{};
            command.disable_arm_motion_planning = step % 2 == 0; // WAQ owns all arm actions.
            command.forward_vel_scale = step < 20 ? 0.f : step < 120 ? 2.f : -2.f;
            command.side_vel_scale = step < 20 ? 0.f : step < 120 ? 2.f : -2.f;
            command.turnning_vel_scale = step < 120 ? -2.f : 2.f;
            const Vec3f requested(command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale);
            const Vec3f increments(.02f,.02f,.04f);
            for (int i = 0; i < 3; ++i)
                filtered(i) += std::clamp(std::clamp(requested(i), -1.f, 1.f) - filtered(i),
                    -increments(i), increments(i));
            std::array<float,96> frame{};
            const Vec3f gravity = state.base_rot_mat.transpose() * Vec3f(0,0,-1);
            for (int i = 0; i < 3; ++i) {
                frame[i] = state.base_omega(i); frame[3+i] = gravity(i);
                const float velocity =
                    std::clamp(filtered(i) * limits(i),
                               std::min(0.f, minimum(i)), std::max(0.f, limits(i)));
                frame[6+i] = std::abs(velocity) < .1f ? 0.f : velocity;
            }
            for (int i = 0; i < 29; ++i) {
                frame[9+i] = state.joint_pos(joints[i]) - defaults[i];
                frame[38+i] = .05f * state.joint_vel(joints[i]);
                frame[67+i] = previous[i];
            }
            for (auto& value : frame) value = std::clamp(value, -100.f, 100.f);
            if (history.empty()) history.assign(5, frame);
            else { history.pop_front(); history.push_back(frame); }
            std::array<float,480> observation{};
            for (int i = 0; i < 5; ++i) std::copy(history[i].begin(), history[i].end(), observation.begin() + i*96);
            auto tensor = Ort::Value::CreateTensor<float>(memory, observation.data(), observation.size(), shape.data(), shape.size());
            auto result = session.Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, 1);
            const VecXf raw = pro_policy::ReadAction(result, 29);
            previous.fill(0.f);
            for (int i = 0; i < 29; ++i) previous[i] = raw(i);
            const auto actual = runner.GetRobotAction(state, command);
            Require(runner.IsReady(), runner.GetLastError());
            for (int i = 0; i < 29; ++i) {
                const int index = i;
                Require(std::abs(actual.goal_joint_pos(joints[index]) -
                    (defaults[index] + .25f * std::clamp(previous[index], -100.f, 100.f))) < 2e-5f,
                    "WAQ reference action mismatch at step " + std::to_string(step));
            }
            for (int i = 0; i < 29; ++i)
                Require(actual.kp(joints[i]) == kp[i] && actual.kd(joints[i]) == kd[i], "WAQ B2 PD mapping mismatch");
        }
        runner.OnExit();
    }
}

int main() {
    try {
        CheckPolicy();
        AmpWalkingWAQPolicyRunner runner("waq-limits", RobotName::CR1PRO);
        bool rejected = false;
        try { runner.SetDecimation(10); } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "WAQ accepted a wrong control period");
        rejected = false;
        try { runner.SetCmdMaxVel(Vec3f(-1,0,0)); } catch (const std::invalid_argument&) { rejected = true; }
        Require(rejected, "WAQ accepted negative velocity limits");
        std::cout << "Walk-to-run 480 -> 29: 720 ONNX reference comparisons, re-entry and asymmetric velocity/period limits passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
