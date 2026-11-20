/**
 * @file model_inference_base.hpp
 * @brief model inference base
 * @author WShzm
 * @version 1.0
 * @date 2025-12-18
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */

#ifndef MODEL_INFERENCE_BASE_HPP_
#define MODEL_INFERENCE_BASE_HPP_

#include "common_types.h"
#include "custom_types.h"
#include "dr_logger.hpp"
#include "parameters/control_parameters.h"

using namespace types;

namespace humanoid {

class ModelInferenceBase {
private:

protected:

public:

    const std::string policy_name_;
    const RobotName robot_name_;
    int decimation_;
    std::unique_ptr<DrLogger> logger_rlcontrol;
    std::shared_ptr<ControlParameters> cp_ptr_;

    ModelInferenceBase(std::string policy_name,  RobotName robot_name) : policy_name_(policy_name), robot_name_(robot_name), logger_rlcontrol(std::make_unique<DrLogger>("POLICY_INFO")) 
    {

    }

    ~ModelInferenceBase() {}

    virtual void OnEnter() = 0;

    virtual void OnExit() = 0;

    virtual RobotAction GetRobotAction(RobotBasicState &, const UserCommand &, MatXf &) = 0;

    virtual void DisplayPolicyInfo() {}

    virtual void DisplayAllPolicyInfo() {}

    /**
     * @brief 步态事件驱动的内部策略切换（仅 External 子类实现）
     */
    virtual void update_inner_policy(bool /*has_gait_event*/, uint32_t /*new_gait*/, RobotBasicState & /*rbs*/) {}
    
    virtual void GetRobotParameters(std::shared_ptr<ControlParameters> cp_ptr) {
        cp_ptr_ = cp_ptr;
    }

    /**
     * @brief Set the decimation
     * @param  d decimation
     */
    virtual void SetDecimation(int d) {
        decimation_ = d;
    }

};

}  // namespace humanoid

#endif
