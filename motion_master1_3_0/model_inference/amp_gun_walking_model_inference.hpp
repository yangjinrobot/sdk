#ifndef AMP_GUN_WALKING_MODEL_INFERENCE_HPP_
#define AMP_GUN_WALKING_MODEL_INFERENCE_HPP_

#include "model_inference_base.hpp"
#include "amp_gun_walking_policy_runner.hpp"

class AmpGunWalkingModelInference : public ModelInferenceBase {
public:
    explicit AmpGunWalkingModelInference(RobotName robot_name)
        : ModelInferenceBase("amp_gun_walking", robot_name), runner_(robot_name) {
        decimation_ = runner_.decimation_;
        runner_.DisplayPolicyInfo();
    }
    void OnEnter() override { runner_.OnEnter(); }
    void OnExit() override { runner_.OnExit(); }
    RobotAction GetRobotAction(RobotBasicState& robot, const UserCommand& command, MatXf& joint_command) override {
        RobotAction action = runner_.GetRobotAction(robot, command);
        joint_command = action.ConvertToMat();
        return action;
    }
private:
    AmpGunWalkingPolicyRunner runner_;
};

#endif