#include "tangxia_policy_runner.h"
#include "zhanli_policy_runner.h"

#include <array>
#include <limits>
#include <stdexcept>

namespace {
void Require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

RobotBasicState Feedback() {
    RobotBasicState state{};
    state.base_rot_mat = RpyToRm(Vec3f(0.8f, -0.6f, 0.2f));
    state.base_omega << 0.1f, -0.2f, 0.3f;
    state.joint_pos = VecXf::Zero(31);
    state.joint_vel = VecXf::Zero(31);
    state.joint_tau = VecXf::Zero(31);
    return state;
}

template <typename Runner>
void CheckRunner(const char* name, int last_frame) {
    Runner runner(name, RobotName::CR1PRO);
    Require(runner.IsReady(), std::string(name) + " failed to load: " + runner.GetLastError());
    Require(runner.decimation_ == 20, std::string(name) + " is not running at 50 Hz");
    Require(runner.LastFrame() == last_frame, std::string(name) + " terminal frame mismatch");

    constexpr std::array<int, 29> policy_to_robot{
        17, 23, 0, 18, 24, 1, 19, 25, 2, 20, 26, 3, 10, 21, 27,
        4, 11, 22, 28, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16};
    constexpr std::array<float, 29> kp_policy{
        100, 100, 80, 100, 100, 80, 80, 80, 100, 100, 100, 100, 100,
        70, 70, 100, 100, 30, 30, 100, 100, 100, 100, 30, 30, 30, 30, 30, 30};
    constexpr std::array<float, 29> kd_policy{
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2,
        3, 3, 1, 1, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1};
    VecXf kp = VecXf::Zero(29), kd = VecXf::Zero(29);
    for (int i = 0; i < 29; ++i) {
        kp(policy_to_robot[i]) = kp_policy[i];
        kd(policy_to_robot[i]) = kd_policy[i];
    }

    auto state = Feedback();
    UserCommand command{};
    runner.OnEnter();
    for (int frame = 0; frame < last_frame + 5; ++frame) {
        const auto action = runner.GetRobotAction(state, command);
        pro_policy::ValidateRobotAction(action);
        Require(runner.IsReady(), std::string(name) + " inference failed: " + runner.GetLastError());
        Require(action.kp.isApprox(kp, 0.f), std::string(name) + " mapped PD kp incorrectly");
        Require(action.kd.isApprox(kd, 0.f), std::string(name) + " mapped PD kd incorrectly");
        Require(action.goal_joint_pos.allFinite() && action.goal_joint_vel.isZero() &&
                    action.tau_ff.isZero(),
                std::string(name) + " produced an invalid hybrid command");
        Require(runner.run_cnt_ == std::min(frame + 1, last_frame),
                std::string(name) + " did not clamp the reference at the terminal frame");
    }
    runner.OnExit();

    runner.OnEnter();
    state.joint_pos(0) = std::numeric_limits<float>::quiet_NaN();
    const auto damping = runner.GetRobotAction(state, command);
    Require(runner.HasInferenceFault() && !runner.IsReady() &&
                damping.kp.isZero() && damping.kd.isOnes(),
            std::string(name) + " accepted a non-finite observation");
    runner.OnEnter();
    Require(runner.IsReady(), std::string(name) + " did not recover on re-entry");
    runner.OnExit();

    Runner missing(std::string(name) + "-missing", RobotName::CR1PRO,
                   "/tmp/nonexistent_recovery_policy.onnx");
    Require(!missing.IsReady(), std::string(name) + " accepted a missing model");
    std::cout << name << ": full trajectory, metadata PD mapping, terminal hold and faults passed\n";
}
}  // namespace

int main() {
    try {
        CheckRunner<TangxiaPolicyRunner>("tangxia", 233);
        CheckRunner<ZhanliPolicyRunner>("zhanli", 316);
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
