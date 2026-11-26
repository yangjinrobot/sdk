// Offline real-model observation, history, re-entry and joint mapping checks.
#include "amp_control_gun_shoot_walk_mode_policy_runner.h"
#include "amp_control_walk2run_mode_policy_runner.h"
#include <cassert>
#include <deque>
#include <fstream>
#include <iomanip>

template <class Runner>
void CheckPolicy(Runner& p, const std::string& mode, const char* trace_path) {
    const int dim = (mode == "1023" || mode == "1026" ||
                     mode == "1026_walk" || mode == "1027" ||
                     mode == "1101" || mode == "1102" ||
                     mode == "1103" || mode == "1103_2" ||
                     mode == "1106" || mode == "1111" ||
                     mode == "1112") ? 73 : 72;
    assert(p.obs_dim_ == dim && p.obs_total_dim_ == dim * 6 && p.decimation_ == 20);
    std::ofstream trace(trace_path);
    trace << std::setprecision(9);
    for (const auto& row : std::vector<std::pair<std::string, VecXf>>{
            {"defaults", p.dof_pos_default_}, {"kp", p.kp_}, {"kd", p.kd_},
            {"max_cmd", p.max_cmd_vel_}})
        trace << row.first << ' ' << row.second.transpose() << '\n';
    RobotBasicState state;
    state.base_rot_mat = Eigen::AngleAxisf(.1f, Vec3f::UnitX()).toRotationMatrix();
    state.base_omega << .1f, -.2f, .3f;
    for (int episode = 0; episode < 2; ++episode) {
        p.OnEnter();
        assert(p.last_action_.isZero() && p.observation_history_.isZero());
        std::deque<VecXf> history;
        VecXf previous = VecXf::Zero(21);
        for (int step = 0; step < 140; ++step) {
            state.joint_pos = VecXf::Zero(21);
            state.joint_vel = VecXf::Zero(21);
            for (int j = 0; j < 21; ++j) {
                state.joint_pos(p.robot2policy_idx[j]) = p.dof_pos_default_(j) + .001f * (j + step % 5);
                state.joint_vel(p.robot2policy_idx[j]) = .02f * (j - step % 3);
            }
            UserCommand cmd{};
            cmd.forward_vel_scale = episode == 0 ? 2.f : -2.f;
            cmd.side_vel_scale = 2.f;
            cmd.turnning_vel_scale = -2.f;
            auto action = p.GetRobotAction(state, cmd);
            assert(p.current_observation_.tail(21).isApprox(previous));
            if (step == 0) history.assign(5, p.current_observation_);
            else { history.pop_front(); history.push_back(p.current_observation_); }
            for (int frame = 0; frame < 5; ++frame)
                assert(p.observation_total_.segment(frame * dim, dim).isApprox(history[frame]));
            assert(p.observation_total_.tail(dim).isApprox(p.current_observation_));
            for (int j = 0; j < 21; ++j) {
                int k = p.policyandpd2robot_idx[j];
                assert(std::fabs(action.goal_joint_pos(j) - (p.dof_pos_default_(k) + .25f * p.action_(k))) < 1e-6f);
                assert(action.kp(j) == p.kp_(k) && action.kd(j) == p.kd_(k));
            }
            assert(p.action_.allFinite() && p.action_.cwiseAbs().maxCoeff() <= 10.f);
            trace << "sample " << p.observation_total_.transpose() << ' ' << p.action_.transpose() << '\n';
            previous = p.action_;
        }
        p.OnExit();
    }
    if (dim == 73) {
        p.SetCmdMaxVel(Vec3f::Ones());
        const float threshold = (mode == "1027" ||
                                 mode == "1101" ||
                                 mode == "1102" ||
                                 mode == "1103" ||
                                 mode == "1103_2" ||
                                 mode == "1106" ||
                                 mode == "1111" ||
                                 mode == "1112") ? .1f : .15f;
        assert(p.moving_flag_threshold_ == threshold);
        for (const Vec3f velocity : std::vector<Vec3f>{
                Vec3f::Zero(), Vec3f(threshold, 0, 0), Vec3f(0, -threshold, 0),
                Vec3f(0, 0, threshold), Vec3f(0, 0, -threshold),
                Vec3f(threshold + .001f, 0, 0), Vec3f(0, -threshold - .001f, 0),
                Vec3f(0, 0, -threshold - .001f), Vec3f(.75f * threshold, .75f * threshold, 0)}) {
            p.OnEnter();
            p.cmd_vel_input_ = velocity;
            UserCommand cmd{};
            cmd.forward_vel_scale = velocity(0);
            cmd.side_vel_scale = velocity(1);
            cmd.turnning_vel_scale = velocity(2);
            p.GetRobotAction(state, cmd);
            const bool moving = velocity.head<2>().norm() > threshold || std::fabs(velocity(2)) > threshold;
            assert(p.current_observation_(9) == (moving ? 1.f : 0.f));
        }
    }
    std::cout << "PASS: " << dim * 6 << "-D history, re-entry, previous action and joint mapping\n";
}

int main(int argc, char** argv) {
    assert(argc == 4);
    const std::string mode = argv[1];
    if (mode == "gun" || mode == "1026_walk") {
        AmpControlGunShootWalkModePolicyRunner policy(mode == "gun" ? "1022_gun" : "1026_walk",
                                       RobotName::CR1STANDARD, argv[2],
                                       PolicyObservationConfig{mode == "gun" ? 72 : 73, 5, true});
        CheckPolicy(policy, mode, argv[3]);
        return 0;
    }
    std::unique_ptr<AmpControlWalk2RunModePolicyRunner> policy;
    if (mode == "1101")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1101_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1101);
    else if (mode == "1102")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1102_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1102);
    else if (mode == "1103")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1103_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1103);
    else if (mode == "1103_2")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1103_2_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1103_2);
    else if (mode == "1106")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1106_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1106);
    else if (mode == "1111")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1111_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1111);
    else if (mode == "1112")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1112_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1112);
    else if (mode == "1027")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1027_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1027);
    else if (mode == "1026")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1026_run", RobotName::CR1STANDARD, argv[2], AmpControlWalk2RunModePolicyRunner::Variant::Run1026);
    else if (mode == "1023")
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1023_humanoid", RobotName::CR1STANDARD, argv[2],
            PolicyObservationConfig{73, 5, true});
    else
        policy = std::make_unique<AmpControlWalk2RunModePolicyRunner>(
            "1022_humanoid", RobotName::CR1STANDARD, argv[2],
            PolicyObservationConfig{72, 5, true});
    CheckPolicy(*policy, mode, argv[3]);
}
