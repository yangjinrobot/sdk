// Exercise the production RL adapter without DDS or hardware.
#include "state_base.h"
#include "chiqiangwalk_policy_runner.h"
#include "crx_model_inference_external.hpp"
#include <cassert>
#include <limits>

int main(int argc, char** argv) {
    assert(argc == 2);
    assert(std::string(CRXModelInference::kPolicyOnnxPath) ==
           "/opt/robot/share/motion_master/policy/amp/1118/nirenwalk.onnx");
    // 15500 remains available through explicit runner injection; the default
    // path constructor now selects the legacy 78-D gait policy.
    auto runner = std::make_shared<ChiqiangWalkPolicyRunner>(
        "rl_walk_1026", RobotName::CR1STANDARD, argv[1],
        PolicyObservationConfig{73, 5, true});
    CRXModelInferenceExternal model("rl_walk_1026", RobotName::CR1STANDARD, runner);
    model.GetRobotParameters(std::make_shared<ControlParameters>(RobotName::CR1STANDARD));
    auto& p = *runner;
    assert(p.obs_total_dim_ == 438 && p.decimation_ == 20);
    assert(p.dof_pos_default_(17) == -.2f && p.dof_pos_default_(18) == -.2f);
    RobotBasicState state;
    state.base_rot_mat.setIdentity();
    state.base_omega.setZero();
    state.joint_pos = VecXf::Zero(21);
    state.joint_vel = VecXf::Zero(21);
    for (int i = 0; i < 21; ++i)
        state.joint_pos(p.robot2policy_idx[i]) = p.dof_pos_default_(i);
    for (float target : {1.f, -1.f}) {
        model.OnEnter();
        assert(StateBase::msfb_.GetCurrentGait() == RobotGaitType::HumanWALK);
        assert(p.last_action_.isZero() && p.observation_history_.isZero());
        UserCommand command{};
        command.forward_vel_scale = target;
        MatXf joints;
        for (int step = 0; step < 60; ++step) {
            auto action = model.GetRobotAction(state, command, joints);
            assert(joints.rows() == 21 && joints.cols() == 5 && joints.allFinite());
            assert(joints.isApprox(action.ConvertToMat()));
        }
        assert(std::fabs(p.current_observation_(6) - (target > 0 ? 1.5f : -1.f)) < 1e-5f);
        command.forward_vel_scale = std::numeric_limits<float>::infinity();
        bool rejected = false;
        try { model.GetRobotAction(state, command, joints); }
        catch (const std::runtime_error&) { rejected = true; }
        assert(rejected);
        model.OnExit();
    }
    std::cout << "PASS: RL adapter 15500 contract, velocity, feedback, re-entry and invalid command\n";
}
