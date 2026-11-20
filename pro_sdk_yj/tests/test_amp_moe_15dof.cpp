#include "amp_moe_15dof_policy_runner.hpp"
#include "rl_stand_policy_runner.hpp"
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
    // Expected order and defaults are independently recorded from the training
    // configuration. The observation and action heads have different orders;
    // the free-base physics regression also checks this mapping end to end.
    const std::array<int, 29> joints{17,23,0,18,24,1,19,25,2,20,26,3,10,21,27,
        4,11,22,28,5,12,6,13,7,14,8,15,9,16};
    const std::array<int, 15> controls{2,5,8,0,3,6,9,13,17,1,4,7,10,14,18};
    const std::array<float, 29> defaults{-.15f,-.15f,0,0,0,0,0,0,0,.3f,.3f,0,0,
        -.15f,-.15f,.2f,-.2f,0,0,0,0,.8f,.8f,0,0,0,0,0,0};
    const std::array<float, 29> kp{350,350,350,350,350,350,200,200,350,350,350,
        200,200,100,100,150,150,100,100,150,150,200,200,150,150,150,150,150,150};
    const std::array<float, 29> kd{10,10,15,10,10,15,5,5,15,10,10,
        5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5,5};
    Ort::Env env(ORT_LOGGING_LEVEL_ERROR, "MoE-reference");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    options.SetGraphOptimizationLevel(ORT_ENABLE_ALL);
    Ort::Session session(env, pro_policy::ResolveModelPath("amp_moe_policy/moe_policy_1026.onnx").c_str(), options);
    pro_policy::ValidateModelSchema(session, 576, 15, "observations", "actions");
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    const char* inputs[]{"observations"}; const char* outputs[]{"actions"};
    const std::array<int64_t, 2> shape{1,576};
    AmpMoe15DofPolicyRunner runner("moe-regression", RobotName::CR1PRO);
    Require(runner.IsReady(), runner.GetLastError());
    Require(runner.decimation_ == 20, "MoE policy period mismatch");
    for (int entry = 0; entry < 2; ++entry) {
        runner.OnEnter();
        std::deque<std::array<float,96>> history;
        std::array<float,29> previous{};
        Vec3f filtered = Vec3f::Zero();
        for (int step = 0; step < 240; ++step) {
            const auto state = Feedback(step);
            UserCommand command{};
            command.disable_arm_motion_planning = true;
            command.forward_vel_scale = step < 20 ? 0.f : step < 120 ? 2.f : -2.f;
            command.side_vel_scale = step < 20 ? 0.f : .4f;
            command.turnning_vel_scale = step < 120 ? -.2f : .6f;
            const Vec3f requested(command.forward_vel_scale, command.side_vel_scale, command.turnning_vel_scale);
            const Vec3f increments(.02f,.02f,.04f);
            for (int i = 0; i < 3; ++i)
                filtered(i) += std::clamp(std::clamp(requested(i), -1.f, 1.f) - filtered(i),
                    -increments(i), increments(i));
            std::array<float,96> frame{};
            const Vec3f gravity = state.base_rot_mat.transpose() * Vec3f(0,0,-1);
            for (int i = 0; i < 3; ++i) {
                frame[i] = state.base_omega(i); frame[3+i] = gravity(i);
                frame[6+i] = filtered(i) * (i < 2 ? .8f : 1.57f);
            }
            for (int i = 0; i < 29; ++i) {
                frame[9+i] = state.joint_pos(joints[i]) - defaults[i];
                frame[38+i] = .05f * state.joint_vel(joints[i]);
                frame[67+i] = previous[i];
            }
            for (auto& value : frame) value = std::clamp(value, -100.f, 100.f);
            if (history.empty()) history.assign(5, frame);
            else { history.pop_front(); history.push_back(frame); }
            std::array<float,576> observation{};
            for (int i = 0; i < 5; ++i) std::copy(history[i].begin(), history[i].end(), observation.begin() + i*96);
            std::copy(frame.begin(), frame.end(), observation.begin() + 480);
            auto tensor = Ort::Value::CreateTensor<float>(memory, observation.data(), observation.size(), shape.data(), shape.size());
            auto result = session.Run(Ort::RunOptions{nullptr}, inputs, &tensor, 1, outputs, 1);
            const VecXf raw = pro_policy::ReadAction(result, 15);
            previous.fill(0.f);
            for (int i = 0; i < 15; ++i) previous[controls[i]] = std::clamp(raw(i), -100.f, 100.f);
            const auto actual = runner.GetRobotAction(state, command);
            Require(runner.IsReady(), runner.GetLastError());
            for (int i = 0; i < 15; ++i) {
                const int index = controls[i];
                Require(std::abs(actual.goal_joint_pos(joints[index]) -
                    (defaults[index] + .25f * previous[index])) < 2e-5f,
                    "MoE reference action mismatch at step " + std::to_string(step));
            }
            Require(actual.goal_joint_pos.segment(3,14).isApprox(state.joint_pos.segment(3,14)),
                    "externally planned arms changed unexpectedly");
            for (int i = 0; i < 29; ++i)
                Require(actual.kp(joints[i]) == kp[i] && actual.kd(joints[i]) == kd[i], "MoE training PD mapping mismatch");
        }
        runner.OnExit();
    }
}

static void CheckArmsAgainstRLStand() {
    AmpMoe15DofPolicyRunner moe("moe-arms", RobotName::CR1PRO);
    RLStandPolicyRunner stand("stand-arms", RobotName::CR1PRO);
    Require(moe.IsReady() && stand.IsReady(), "arm reference policies failed to load");
    stand.SetDecimation(20);  // Compare both planners at the same control period.
    const auto state = Feedback(0);
    const Vec7f left = Vec7f::Constant(.12f), right = Vec7f::Constant(-.08f);
    moe.SetInitialArmTargets(left,right); stand.SetInitialArmTargets(left,right);
    moe.OnEnter(); stand.OnEnter();
    UserCommand command{};
    for (int step = 0; step < 900; ++step) {
        if (step % 150 == 10) {
            command.arm_preset = step / 150 + 1;
            ++command.arm_preset_seq;
        }
        if (step == 61) command.disable_arm_motion_planning = true;
        if (step == 65) command.disable_arm_motion_planning = false;
        const auto actual = moe.GetRobotAction(state,command);
        const auto expected = stand.GetRobotAction(state,command);
        Require(moe.IsReady() && stand.IsReady(), "arm reference inference fault");
        Require(actual.goal_joint_pos.segment(3,14).isApprox(expected.goal_joint_pos.segment(3,14), 1e-6f),
                "MoE arm interpolation differs from RL Stand at step " + std::to_string(step));
    }
    moe.OnExit(); stand.OnExit();
}

int main() {
    try {
        CheckPolicy();
        CheckArmsAgainstRLStand();
        std::cout << "MoE 15 DOF: 480 ONNX reference comparisons and 900 RL Stand arm comparisons passed\n";
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
