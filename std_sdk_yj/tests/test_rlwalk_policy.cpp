// Regression: AmpControl_GunShootWalk_Mode 1118 deployment defaults.
#include "amp_walk_policy_runner.h"
#include <cassert>

int main(int argc, char** argv) {
    assert(argc == 2);
    AmpWalkPolicyRunner policy(
        "rlwalk_1118", RobotName::CR1STANDARD, argv[1]);

    VecXf expected(21);
    expected <<
        -0.1f, -0.1f, 0.0f, 0.0f, 0.0f,
         0.0f,  0.0f, 0.0f, 0.0f, 0.15f, -0.15f,
         0.2f,  0.2f, 0.0f, 0.0f, -0.1f, -0.1f,
         1.2f,  1.2f, 0.0f, 0.0f;
    assert(policy.obs_dim_ == 78 && policy.obs_total_dim_ == 468);
    assert(policy.act_dim_ == 21 && policy.decimation_ == 20);
    assert(policy.action_scale_ == 0.25f && policy.action_clip_ == 10.0f);
    assert((policy.max_cmd_vel_ - Vec3f(1.0f, 0.6f, 1.0f)).cwiseAbs().maxCoeff() < 1e-7f);
    assert(std::fabs(policy.reverse_velocity_scale_ - 0.6f) < 1e-7f);
    assert((policy.dof_pos_default_ - expected).cwiseAbs().maxCoeff() < 1e-7f);

    RobotBasicState state;
    state.base_rot_mat.setIdentity();
    state.base_omega.setZero();
    state.joint_pos = VecXf::Zero(21);
    state.joint_vel = VecXf::Zero(21);
    for (int i = 0; i < 21; ++i)
        state.joint_pos(policy.robot2policy_idx[i]) = expected(i);

    policy.OnEnter();
    const RobotAction action = policy.GetRobotAction(state, UserCommand{});
    assert(action.goal_joint_pos.allFinite() && action.kp.allFinite() && action.kd.allFinite());
    for (int frame = 0; frame < 4; ++frame)
        assert(policy.observation_total_.segment(frame * 78, 78).isApprox(
            policy.observation_total_.tail(78)));
    for (int i = 0; i < 21; ++i) {
        const int p = policy.policyandpd2robot_idx[i];
        assert(std::fabs(action.goal_joint_pos(i) -
                         (expected(p) + 0.25f * policy.action_(p))) < 1e-5f);
    }

    for (const auto& [input, expected_vx] :
         std::initializer_list<std::pair<float, float>>{{1.0f, 1.0f}, {-1.0f, -0.6f}}) {
        policy.OnEnter();
        UserCommand command{};
        command.forward_vel_scale = input;
        for (int step = 0; step < 60; ++step)
            policy.GetRobotAction(state, command);
        assert(std::fabs(policy.current_observation_(6) - expected_vx) < 1e-5f);
    }

    policy.OnEnter();
    UserCommand command{};
    command.forward_vel_scale = -0.15f;  // -0.09 m/s
    command.side_vel_scale = 0.16f;      //  0.096 m/s
    command.turnning_vel_scale = -0.099f;
    for (int step = 0; step < 12; ++step) {
        policy.GetRobotAction(state, command);
        assert(policy.current_observation_.segment(6, 3).isZero(0));
    }

    command.forward_vel_scale = 0.1f;
    command.turnning_vel_scale = -0.11f;
    for (int step = 0; step < 15; ++step)
        policy.GetRobotAction(state, command);
    assert(policy.current_observation_(6) == 0.1f);
    assert(policy.current_observation_(7) == 0.0f);
    assert(policy.current_observation_(8) == -0.11f);

    command = UserCommand{};
    policy.GetRobotAction(state, command);
    assert(policy.current_observation_.segment(6, 3).isZero(0));
    return 0;
}
