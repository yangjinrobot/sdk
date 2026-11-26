// AmpControl_NiRenWalk_Mode deployment check: policy_15500 uses the STD
// 78 x 6 observation layout and the NiRenWalk default posture.
#include "amp_control_ni_ren_walk_mode_policy_runner.h"

#include <cassert>
#include <cmath>

int main(int argc, char** argv) {
    assert(argc == 2);
    AmpControlNiRenWalkModePolicyRunner policy(
        "rl_control_policy_15500", RobotName::CR1STANDARD, argv[1]);

    assert(policy.obs_dim_ == 78);
    assert(policy.obs_history_num_ == 5);
    assert(policy.obs_total_dim_ == 468);
    assert(policy.act_dim_ == 21);
    assert(policy.decimation_ == 20);
    assert((policy.max_cmd_vel_ - Vec3f(1.2f, 0.5f, 1.0f)).cwiseAbs().maxCoeff() < 1e-7f);
    assert(std::fabs(policy.reverse_velocity_scale_ - 0.5f) < 1e-7f);
    assert(policy.dof_pos_default_(17) == 1.2f && policy.dof_pos_default_(18) == 1.2f);

    RobotBasicState robot;
    robot.base_rot_mat.setIdentity();
    robot.base_omega.setZero();
    robot.joint_pos = VecXf::Zero(21);
    robot.joint_vel = VecXf::Zero(21);
    policy.OnEnter();
    const RobotAction action = policy.GetRobotAction(robot, UserCommand{});
    assert(action.goal_joint_pos.allFinite());
    assert(action.kp.allFinite() && action.kd.allFinite());
    for (int frame = 0; frame < 6; ++frame)
        assert(policy.observation_total_.segment(frame * 78, 78).isApprox(
            policy.current_observation_));
    return 0;
}
