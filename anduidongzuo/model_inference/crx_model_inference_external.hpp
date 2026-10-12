/**
 * @file crx_model_inference_external.hpp
 * @brief crx model inference external — 仅使用 CommonPolicyRunner
 * @author WShzm
 * @version 2.0
 * @date 2025-12-18
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#ifndef CRX_MODEL_INFERENCE_EXTERNAL_HPP_
#define CRX_MODEL_INFERENCE_EXTERNAL_HPP_

#include "crx_model_inference.hpp"

class CRXModelInferenceExternal : public CRXModelInference {
public:
    std::string GetCurrentPolicyName() const { return inner_ptr_ ? inner_ptr_->policy_name_ : "null"; }

    CRXModelInferenceExternal(const std::string &policy_name, RobotName robot_name_,
                              const std::string &onnx_path = kPolicyOnnxPath) :
        CRXModelInference(policy_name, robot_name_, onnx_path) {}

    CRXModelInferenceExternal(const std::string &policy_name, RobotName robot_name_,
                              std::shared_ptr<CommonPolicyRunner> policy_runner) :
        CRXModelInference(policy_name, robot_name_, std::move(policy_runner)) {}

    void SetPdGains(const VecXf& kp, const VecXf& kd) {
        common_policy_->SetPdGains(kp, kd);
    }

    void update_inner_policy(bool has_gait_event, uint32_t new_gait, RobotBasicState &rbs_) {
        if (!has_gait_event) return;
        if (new_gait == StateBase::msfb_.GetCurrentGait()) return;
        std::cout << "new_gait: " << (int)new_gait << std::endl;
        logger_rlcontrol->info("gait event consumed: 0x%x, current: 0x%x", new_gait, StateBase::msfb_.GetCurrentGait());
        StateBase::msfb_.UpdateCurrentGait(new_gait);
    }

    void OnExit() {
        inner_ptr_->OnExit();
    }

    void OnEnter() {
        inner_ptr_ = common_policy_;
        SetDecimation(inner_ptr_->decimation_);
        inner_ptr_->OnEnter();
        StateBase::msfb_.UpdateCurrentGait(RobotGaitType::HumanWALK);
    }

    RobotAction GetRobotAction(RobotBasicState &ro, const UserCommand &uc, MatXf &joint_cmd) {
        RobotAction ra = inner_ptr_->GetRobotAction(ro, uc);
        joint_cmd = postprocess(ra, uc);
        return ra;
    }
};

#endif  // CRX_POLICY_RUNNER_EXTERNAL_HPP_
