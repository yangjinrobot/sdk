/**
 * @file crx_model_inference.hpp
 * @brief crx model inference
 * @author WShzm
 * @version 1.0
 * @date 2025-12-18
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#ifndef CRX_MODEL_INFERENCE_HPP_
#define CRX_MODEL_INFERENCE_HPP_

#include "model_inference_base.hpp"
#include "../run_policy/cr1/include/common_policy_runner.h"

namespace humanoid {

class CRXModelInference : public ModelInferenceBase {
public:
    static constexpr const char* kPolicyOnnxPath = "/opt/robot/share/motion_master/policy/policy_CR1_8.onnx";

    std::shared_ptr<PolicyRunnerBase> inner_ptr_;
    std::shared_ptr<CommonPolicyRunner> common_policy_;

    CRXModelInference(const std::string &policy_name, RobotName robot_name_) : ModelInferenceBase(policy_name, robot_name_) {
        common_policy_ = std::make_shared<CommonPolicyRunner>("common", robot_name_, kPolicyOnnxPath);
        common_policy_->SetCmdMaxVel(Vec3f(1.0, 0.5, 2.0));

        inner_ptr_ = common_policy_;
        SetDecimation(inner_ptr_->decimation_);
        if (!inner_ptr_) {
            std::cerr << "inner_ptr_: error policy" << std::endl;
            exit(0);
        }
    }
    // qxj pro add 新增：注入 runner（PRO 用） 6
    CRXModelInference(const std::string& policy_name, RobotName robot_name_,
                      std::shared_ptr<PolicyRunnerBase> runner)
        : ModelInferenceBase(policy_name, robot_name_), inner_ptr_(std::move(runner)) {
        if (!inner_ptr_) { std::cerr << "[FATAL] null runner" << std::endl; exit(1); }
        SetDecimation(inner_ptr_->decimation_);
    }

    void OnEnter() {
        SetDecimation(inner_ptr_->decimation_);
        inner_ptr_->OnEnter();
    }

    void OnExit() {
        inner_ptr_->OnExit();
    }

    void DisplayPolicyInfo() {
        inner_ptr_->DisplayPolicyInfo();
    }

    void GetRobotParameters(std::shared_ptr<ControlParameters> cp_ptr) {
        cp_ptr_ = cp_ptr;
    }

    void update_inner_policy(const UserCommand &uc){
        
    };

    // 后处理
    MatXf postprocess(RobotAction& ra, const UserCommand &uc){
        MatXf res = ra.ConvertToMat();

        MatXf joint_cmd = MatXf::Zero(cp_ptr_->dof_num_, 5);
        MatXf waist_joint_cmd = MatXf::Zero(cp_ptr_->waist_dof_num_, 5);
        MatXf arm_joint_cmd   = MatXf::Zero(2*cp_ptr_->arm_dof_num_, 5);
        MatXf neck_joint_cmd  = MatXf::Zero(cp_ptr_->neck_dof_num_, 5);
        MatXf leg_joint_cmd   = MatXf::Zero(2*cp_ptr_->leg_dof_num_, 5);

        arm_joint_cmd.col(0) = cp_ptr_->arm_kp_.replicate(2, 1);
        arm_joint_cmd.col(2) = cp_ptr_->arm_kd_.replicate(2, 1);
        waist_joint_cmd.col(0) = cp_ptr_->waist_kp_;
        waist_joint_cmd.col(2) = cp_ptr_->waist_kd_;
        neck_joint_cmd.col(0) = cp_ptr_->neck_kp_;
        neck_joint_cmd.col(2) = cp_ptr_->neck_kd_;
        if (cp_ptr_->dof_num_ == 31 || cp_ptr_->dof_num_ == 30) {
            // 只替换掉策略输出对应的关节，其余关节按原有pd锁死
            // qxj pro add ** 6
            // waist_joint_cmd.block<1,5>(0,0) = res.block<1,5>(0,0);
            // arm_joint_cmd.block<4,5>(0,0) = res.block<4,5>(1,0);
            // arm_joint_cmd.block<4,5>(7,0) = res.block<4,5>(5,0);
            // leg_joint_cmd = res.block<12,5>(9,0);
            // joint_cmd << waist_joint_cmd, arm_joint_cmd, leg_joint_cmd, neck_joint_cmd;
            joint_cmd << res, neck_joint_cmd;
        }else{
            joint_cmd = res;
        }
        return joint_cmd;
    }

    RobotAction GetRobotAction(RobotBasicState &ro, const UserCommand &uc, MatXf &joint_cmd) {
        return inner_ptr_->GetRobotAction(ro, uc);
    }
        
};

}  // namespace humanoid

#endif // CRX_POLICY_RUNNER_HPP_
