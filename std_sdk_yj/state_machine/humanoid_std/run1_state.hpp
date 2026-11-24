#ifndef HUMANOID_STD_RUN1_STATE_HPP_
#define HUMANOID_STD_RUN1_STATE_HPP_

#include "state_base.h"
#include "run_policy_runner.h"
#include "arm_entry_transition.hpp"

namespace humanoid_std {
// Legacy Run1 controller. It is intentionally not registered in the runtime FSM.
class Run1State final : public StateBase {
public:
    static constexpr const char* kDefaultPolicyPath =
        "/opt/robot/share/motion_master/policy/1103_2/1103_2.onnx";

    Run1State(const RobotName& robot_name, const std::string& state_name,
              std::shared_ptr<ControllerData> data_ptr,
              const std::string& policy_path = kDefaultPolicyPath)
        : StateBase(robot_name, state_name, data_ptr),
          policy_("run_1", robot_name, policy_path,
                  RunPolicyRunner::Variant::Run1103_2) {}

    void OnEnter() override {
        policy_.OnEnter();
        run_count_ = 0;
        failed_ = false;
        joint_cmd_ = ri_ptr_->GetJointCommand();
        if (joint_cmd_.rows() != cp_ptr_->dof_num_ || joint_cmd_.cols() != 5 ||
            !joint_cmd_.allFinite()) {
            std::cerr << "[run_1] invalid previous joint command; entering damping\n";
            failed_ = true;
            return;
        }
        // Preserve the last command published by the previous state, including PD.
        arm_entry_transition_.Reset(joint_cmd_.block(1, 0, 8, 5));
        msfb_.UpdateCurrentState(RobotMotionState::Run1);
        msfb_.UpdateCurrentGait(0);
        std::cout << "[run_1] policy started at 50 Hz" << std::endl;
    }

    void OnExit() override { policy_.OnExit(); }

    void Run() override {
        if (LoseControlJudge()) return;
        if (run_count_ % policy_.decimation_ == 0) {
            RobotBasicState state;
            state.base_rpy = ri_ptr_->GetImuRpy();
            state.base_rot_mat = RpyToRm(state.base_rpy);
            state.base_omega = ri_ptr_->GetImuOmega();
            state.joint_pos = ri_ptr_->GetJointPosition();
            state.joint_vel = ri_ptr_->GetJointVelocity();
            try {
                joint_cmd_ = policy_.GetRobotAction(state, *uc_ptr_->GetUserCommand()).ConvertToMat();
            } catch (const std::exception& error) {
                std::cerr << "[run_1] " << error.what() << "; entering damping" << std::endl;
                failed_ = true;
                return;
            }
        }
        MatXf command = joint_cmd_;
        arm_entry_transition_.Apply(command, 1);
        ri_ptr_->SetJointCommand(command);
        ++run_count_;
    }

    bool LoseControlJudge() override {
        const auto* command = uc_ptr_->GetUserCommand();
        return failed_ || command->safe_control_mode != 0 ||
               command->target_mode == RobotMotionState::JointDamping;
    }

    StateName GetNextStateName() override {
        if (LoseControlJudge()) return StateName::kJointDamping;
        switch (uc_ptr_->GetUserCommand()->target_mode) {
            case RobotMotionState::Run: return StateName::kRun;
            case RobotMotionState::Run2: return StateName::kRun2;
            case RobotMotionState::Run3: return StateName::kRun3;
            case RobotMotionState::Run4: return StateName::kRun4;
            case RobotMotionState::Restand: return StateName::kRestand;
            case RobotMotionState::DunXia: return StateName::kDunXia;
            case RobotMotionState::Stand2SitMimic: return StateName::kStand2SitMimic;
            case RobotMotionState::RLControlMode: return StateName::kRLControl;
            case RobotMotionState::ZeroPos: return StateName::kZeroPos;
            default: return StateName::kRun1;
        }
    }

private:
    ArmEntryTransition arm_entry_transition_;
    RunPolicyRunner policy_;
    MatXf joint_cmd_;
    int run_count_ = 0;
    bool failed_ = false;
};
}  // namespace humanoid_std

#endif
