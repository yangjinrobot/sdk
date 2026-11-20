#include "amp_policy_runner.h"
#include "mimic12_policy_runner.h"
#include "mimic13_policy_runner.h"
#include "mimic1_policy_runner.h"
#include "mimic2_policy_runner.h"
#include "amp_walking_policy_runner.hpp"
#include "amp_walking_waq_policy_runner.hpp"
#include "amp_moe_15dof_policy_runner.hpp"
#include "humanoid_amp_policy_runner.hpp"
#include "rl_stand_policy_runner.hpp"

#include <array>
#include <limits>
#include <stdexcept>

static void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

template <typename Function>
static void RequireThrows(Function function, const char* message) {
    bool threw = false;
    try { function(); } catch (const std::exception&) { threw = true; }
    Require(threw, message);
}

static RobotBasicState State(int dimensions) {
    RobotBasicState state{};
    state.base_rot_mat.setIdentity();
    state.base_omega << .01f, -.02f, .03f;
    state.base_acc << 0.f, 0.f, gravity;
    state.joint_pos = VecXf::LinSpaced(dimensions, -.05f, .05f);
    state.joint_vel = VecXf::Constant(dimensions, .01f);
    state.joint_tau = VecXf::Zero(dimensions);
    return state;
}

static void RequireDamping(const RobotAction& action, const char* message) {
    pro_policy::ValidateRobotAction(action);
    Require(action.kp.isZero() && (action.kd.array() > 0.f).all() &&
            action.tau_ff.isZero() && action.goal_joint_vel.isZero(), message);
}

template <typename Runner>
static void CheckRunner(const char* name, int steps = 24) {
    Runner runner(name, RobotName::CR1PRO);
    Require(runner.IsReady(), std::string(name) + " failed to load: " + runner.GetLastError());
    runner.SetCmdMaxVel(Vec3f(.8f, .5f, .5f));
    UserCommand command{};
    command.forward_vel_scale = .2f;
    command.side_vel_scale = -.1f;
    command.turnning_vel_scale = .05f;
    const auto state29 = State(29);
    auto state31 = State(31);
    state31.joint_pos.head(29) = state29.joint_pos;
    state31.joint_vel.head(29) = state29.joint_vel;
    state31.joint_tau.head(29) = state29.joint_tau;
    state31.joint_pos.tail(2) << .7f, -.6f;
    state31.joint_vel.tail(2) << .2f, -.1f;
    std::vector<MatXf> expected;
    runner.OnEnter();
    for (int step = 0; step < steps; ++step) {
        auto action = runner.GetRobotAction(state29, command);
        Require(runner.IsReady() && !runner.HasInferenceFault(),
                std::string(name) + " inference failed: " + runner.GetLastError());
        pro_policy::ValidateRobotAction(action);
        expected.push_back(action.ConvertToMat());
    }
    runner.OnEnter();
    for (int step = 0; step < steps; ++step) {
        auto action = runner.GetRobotAction(state31, command);
        Require(runner.IsReady() && !runner.HasInferenceFault(),
                std::string(name) + " 31-joint inference failed: " + runner.GetLastError());
        Require(action.ConvertToMat().isApprox(expected[step], 1e-6f),
                std::string(name) + " neck joints changed the 29-joint policy mapping");
    }
    auto bad_size = State(28);
    const auto damping = runner.GetRobotAction(bad_size, command);
    Require(runner.HasInferenceFault() && !runner.IsReady() && !runner.GetLastError().empty(),
            std::string(name) + " did not latch a bad state size");
    RequireDamping(damping, "invalid state did not return damping");
    runner.OnEnter();
    state31.joint_pos(4) = std::numeric_limits<float>::quiet_NaN();
    RequireDamping(runner.GetRobotAction(state31, command), "NaN state did not return damping");
    Require(runner.HasInferenceFault() && !runner.IsReady(), "NaN state was accepted");
    runner.OnEnter();
    const auto valid = State(31);
    runner.GetRobotAction(valid, command);
    Require(runner.IsReady(), "a new state entry did not clear a transient inference fault");
    runner.OnExit();
    std::cout << name << ": model warm-up, 29/31 mapping and failure damping passed\n";
}

template <typename Runner>
static void CheckImportedLoadErrors(const char* path) {
    Runner missing("missing-import", RobotName::CR1PRO, "/nonexistent/imported-policy.onnx");
    Require(!missing.IsReady() && missing.HasInferenceFault(), "Missing imported model was accepted");
    RequireDamping(missing.GetRobotAction(State(31), UserCommand{}), "Missing model did not return damping");
    Runner wrong("wrong-import", RobotName::CR1PRO, pro_policy::ResolveModelPath(path));
    Require(!wrong.IsReady() && wrong.HasInferenceFault(), "Wrong imported model schema was accepted");
}

static void CheckAmpArmEntry() {
    HumanoidAmpPolicyRunner runner("arm-entry", RobotName::CR1PRO);
    Require(runner.IsReady(), "AMP model failed to load");
    const Vec7f left = Vec7f::Constant(.3f), right = Vec7f::Constant(-.2f);
    runner.SetInitialArmTargets(left, right);
    runner.OnEnter();
    const auto action = runner.GetRobotAction(State(31), UserCommand{});
    Require(runner.IsReady(), "AMP arm entry inference failed");
    Require(action.goal_joint_pos.segment<7>(3).isApprox(left) &&
            action.goal_joint_pos.segment<7>(10).isApprox(right), "AMP entry jumped away from previous arm targets");
    runner.OnExit();
}

static void CheckTensorValidation() {
    auto memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::vector<float> values(30, 0.f);
    std::vector<Ort::Value> outputs;
    const std::array<int64_t, 2> wrong_shape{1, 30};
    outputs.emplace_back(Ort::Value::CreateTensor<float>(
        memory, values.data(), values.size(), wrong_shape.data(), wrong_shape.size()));
    RequireThrows([&] { pro_policy::ReadAction(outputs, 29); }, "wrong action shape accepted");
    outputs.clear();
    std::vector<double> doubles(29, 0.0);
    const std::array<int64_t, 2> shape{1, 29};
    outputs.emplace_back(Ort::Value::CreateTensor<double>(
        memory, doubles.data(), doubles.size(), shape.data(), shape.size()));
    RequireThrows([&] { pro_policy::ReadAction(outputs, 29); }, "non-float action accepted");
    outputs.clear();
    values.resize(29);
    values[8] = std::numeric_limits<float>::infinity();
    outputs.emplace_back(Ort::Value::CreateTensor<float>(
        memory, values.data(), values.size(), shape.data(), shape.size()));
    RequireThrows([&] { pro_policy::ReadAction(outputs, 29); }, "non-finite action accepted");
    Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "schema-test");
    Ort::SessionOptions options;
    options.SetIntraOpNumThreads(1);
    const std::string path = pro_policy::ResolveModelPath("amp_policy/amp_1018.onnx");
    Ort::Session session(environment, path.c_str(), options);
    RequireThrows([&] { pro_policy::ValidateModelSchema(session, 480, 29, "obs", "actions"); },
                  "incompatible model observation shape accepted");
}

int main() {
    try {
        Require(RLStandPolicyRunner::GetUdpNode() == nullptr, "test must start without UDP");
        CheckRunner<AmpPolicyRunner>("Imported AMP", 450);
        CheckRunner<Mimic12PolicyRunner>("Mimic12", 450);
        CheckRunner<Mimic13PolicyRunner>("Mimic13", 380);
        CheckRunner<Mimic1PolicyRunner>("Mimic1", 450);
        CheckRunner<Mimic2PolicyRunner>("Mimic2", 380);
        CheckRunner<AmpMoe15DofPolicyRunner>("AMP MoE 15 DOF");
        CheckRunner<AmpWalkingWAQPolicyRunner>("AMP Walking WAQ");
        CheckRunner<AmpWalkingPolicyRunner>("AMP walking");
        CheckRunner<HumanoidAmpPolicyRunner>("AMP whole body");
        CheckRunner<RLStandPolicyRunner>("RL stand");
        Require(RLStandPolicyRunner::GetUdpNode() == nullptr, "runner created a UDP node");
        CheckImportedLoadErrors<AmpPolicyRunner>("mimic_policy/sit2stand.onnx");
        CheckImportedLoadErrors<Mimic12PolicyRunner>("mimic_policy/sit2stand.onnx");
        CheckImportedLoadErrors<Mimic13PolicyRunner>("mimic_policy/model_stand2sit_1020.onnx");
        CheckImportedLoadErrors<Mimic1PolicyRunner>("mimic_policy/sit2stand.onnx");
        CheckImportedLoadErrors<Mimic2PolicyRunner>("mimic_policy/sit2stand.onnx");
        CheckAmpArmEntry();
        CheckTensorValidation();
        const char* configured = std::getenv("MOTION_MASTER_POLICY_DIR");
        const bool had_configured = configured != nullptr;
        const std::string saved = configured ? configured : "";
        setenv("MOTION_MASTER_POLICY_DIR", "/nonexistent/cr1pro-policy-test", 1);
        RequireThrows([] { pro_policy::ResolveModelPath("amp_policy/amp_1018.onnx"); },
                      "explicit missing policy root silently used a fallback");
        AmpWalkingPolicyRunner missing("missing-model", RobotName::CR1PRO);
        Require(!missing.IsReady() && missing.HasInferenceFault() &&
                missing.GetLastError().find("/nonexistent/cr1pro-policy-test") != std::string::npos,
                "missing model did not preserve its load error");
        if (had_configured) setenv("MOTION_MASTER_POLICY_DIR", saved.c_str(), 1);
        else unsetenv("MOTION_MASTER_POLICY_DIR");
        // An invalid address is rejected before socket()/bind(), so this also
        // checks failed-node lifecycle without opening any network socket.
        UdpNode invalid("invalid-address", 8888, "127.0.0.1", 9999);
        Require(!invalid.isRunning() && !invalid.start(), "invalid UDP node was marked running");
        invalid.stop();
        invalid.stop();
        std::cout << "All ten CR1PRO policies passed offline validation\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
