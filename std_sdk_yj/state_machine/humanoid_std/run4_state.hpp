#ifndef HUMANOID_STD_RUN4_STATE_HPP_
#define HUMANOID_STD_RUN4_STATE_HPP_

#include "state_base.h"
#include "run_policy_runner.h"
#include "arm_entry_transition.hpp"

namespace humanoid_std {
// Run4 uses the 1112 walk-to-run policy and motion-12 state code.
class Run4State final : public StateBase {
public:
    static constexpr const char* kDefaultPolicyPath =
        "/opt/robot/share/motion_master/policy/1112/run_1112.onnx";

    Run4State(const RobotName& robot_name, const std::string& state_name,
              std::shared_ptr<ControllerData> data_ptr,
              const std::string& policy_path = kDefaultPolicyPath)
        : StateBase(robot_name, state_name, data_ptr),
          policy_("run_4", robot_name, policy_path,
                  RunPolicyRunner::Variant::Run1112) {}

    void OnEnter() override {
        policy_.OnEnter();
        run_count_ = 0;
        failed_ = false;
        joint_cmd_ = ri_ptr_->GetJointCommand();
        if (joint_cmd_.rows() != cp_ptr_->dof_num_ || joint_cmd_.cols() != 5 ||
            !joint_cmd_.allFinite()) {
            std::cerr << "[run_4] invalid previous joint command; entering damping\n";
            failed_ = true;
            return;
        }
        arm_entry_transition_.Reset(joint_cmd_.block(1, 0, 8, 5));
        msfb_.UpdateCurrentState(RobotMotionState::Run4);
        msfb_.UpdateCurrentGait(0);
        std::cout << "[run_4] 1112 policy started at 50 Hz" << std::endl;
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
                std::cerr << "[run_4] " << error.what() << "; entering damping\n";
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
            case RobotMotionState::RLControlMode: return StateName::kRLControl;
            case RobotMotionState::Run: return StateName::kRun;
            case RobotMotionState::Run2: return StateName::kRun2;
            case RobotMotionState::Run3: return StateName::kRun3;
            case RobotMotionState::Restand: return StateName::kRestand;
            case RobotMotionState::Stand2SitMimic: return StateName::kStand2SitMimic;
            case RobotMotionState::ZeroPos: return StateName::kZeroPos;
            default: return StateName::kRun4;
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
