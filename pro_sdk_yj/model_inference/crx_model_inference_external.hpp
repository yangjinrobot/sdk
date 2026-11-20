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
#include "state_base.hpp"

namespace humanoid {

class CRXModelInferenceExternal : public CRXModelInference {
public:
    std::string GetCurrentPolicyName() const { return inner_ptr_ ? inner_ptr_->policy_name_ : "null"; }

    CRXModelInferenceExternal(const std::string &policy_name, RobotName robot_name_) : 
        CRXModelInference(policy_name, robot_name_) {}
    //qxj pro add 【新增】3 参版：把外部建好的 runner 透传给基类
    CRXModelInferenceExternal(const std::string &policy_name, RobotName robot_name_,
                              std::shared_ptr<PolicyRunnerBase> runner) :
        CRXModelInference(policy_name, robot_name_, runner) {}

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
        // qxj pro add 1
        // inner_ptr_ = common_policy_;
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

}  // namespace humanoid

#endif  // CRX_POLICY_RUNNER_EXTERNAL_HPP_
