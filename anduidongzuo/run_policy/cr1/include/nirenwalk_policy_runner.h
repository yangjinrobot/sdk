#ifndef NIRENWALK_POLICY_RUNNER_H_
#define NIRENWALK_POLICY_RUNNER_H_

#include "common_policy_runner.h"
#include <stdexcept>

// nirenxingzou.onnx 与 NewPolicyRunner 使用相同的 78*(5+1) 观测布局和 21 关节顺序。
// 仅为此模型覆盖 env.yaml 中的默认角度及 PD，不改变现有 RL 策略。
class NirenWalkPolicyRunner final : public CommonPolicyRunner {
public:
    NirenWalkPolicyRunner(const std::string& name, RobotName robot_name,
                          const std::string& path)
        : CommonPolicyRunner(name, robot_name, path, ORT_ENABLE_ALL, {78, 5}) {
        if (robot_name != RobotName::CR1STANDARD)
            throw std::invalid_argument("nirenwalk requires CR1STANDARD (21 DOF)");

        // 与 NewPolicyRunner 的 policy_order 完全一致。
        dof_pos_default_ << -0.2f, -0.2f, 0.0f, 0.0f, 0.0f,
                             0.0f, 0.0f, 0.0f, 0.0f, 0.15f, -0.15f,
                             0.4f, 0.4f, 0.0f, 0.0f, -0.2f, -0.2f,
                             1.2f, 1.2f, 0.0f, 0.0f;
        // env.yaml: legs/feet/arms/body actuator stiffness and damping.
        VecXf kp(21), kd(21);
        kp << 300, 300, 200, 300, 300, 100, 100, 200, 200,
              100, 100, 300, 300, 100, 100, 80, 80, 100, 100, 30, 30;
        kd << 10, 10, 5, 10, 10, 5, 5, 5, 5,
              5, 5, 10, 10, 5, 5, 3, 3, 5, 5, 1, 1;
        SetPdGains(kp, kd);
        SetCmdMaxVel(Vec3f(1.2f, 0.5f, 1.0f));
    }

    RobotAction GetRobotAction(const RobotBasicState& state,
                               const UserCommand& command) override {
        if (state.joint_pos.size() != 21 || state.joint_vel.size() != 21 ||
            !state.joint_pos.allFinite() || !state.joint_vel.allFinite() ||
            !state.base_rot_mat.allFinite() || !state.base_omega.allFinite())
            throw std::runtime_error("nirenwalk: invalid robot observation");
        RobotAction action = CommonPolicyRunner::GetRobotAction(state, command);
        if (!action_.allFinite() || action_.cwiseAbs().maxCoeff() > 10.0f ||
            !action.goal_joint_pos.allFinite() || !action.kp.allFinite() ||
            !action.kd.allFinite())
            throw std::runtime_error("nirenwalk: invalid or out-of-range policy action");
        return action;
    }
};

#endif
