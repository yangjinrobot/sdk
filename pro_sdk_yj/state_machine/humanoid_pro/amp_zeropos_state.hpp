#ifndef HUMANOID_PRO_AMP_ZEROPOS_STATE_HPP_
#define HUMANOID_PRO_AMP_ZEROPOS_STATE_HPP_

#include "state_base.hpp"

namespace humanoid_pro {
class AmpZeroPosState : public StateBase {
private:
    float time_stamp_record_, run_time_;
    VecXf init_joint_pos_, init_joint_vel_, current_joint_pos_, current_joint_vel_;
    VecXf kp_, kd_, goal_joint_pos_;
    MatXf joint_cmd_;

    void GetRobotJointValue() {
        run_time_ = ri_ptr_->GetInterfaceTimeStamp();
        current_joint_pos_ = ri_ptr_->GetJointPosition();
        current_joint_vel_ = ri_ptr_->GetJointVelocity();
    }

    void RecordJointData() {
        time_stamp_record_ = run_time_;
        init_joint_pos_ = current_joint_pos_;
        init_joint_vel_ = current_joint_vel_;
    }

public:
    AmpZeroPosState(const RobotName& robot_name, const std::string& state_name, std::shared_ptr<ControllerData> data_ptr)
        : StateBase(robot_name, state_name, data_ptr) {
        goal_joint_pos_ = VecXf::Zero(cp_ptr_->dof_num_);
        joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
        kp_ = VecXf::Zero(cp_ptr_->dof_num_);
        kd_ = VecXf::Zero(cp_ptr_->dof_num_);

        if (robot_name_ == RobotName::CR1PRO) {
            kp_ << cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_, cp_ptr_->leg_kp_, cp_ptr_->leg_kp_,
                cp_ptr_->neck_kp_;
            kd_ << cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_, cp_ptr_->leg_kd_,
                cp_ptr_->neck_kd_;

                    // 设置默认关节目标位置（29个关节，顺序与控制器内部关节顺序一致）
            goal_joint_pos_.head(29) <<  0.0,   // waist_yaw
                                0.0,   // waist_roll
                                0.0,   // waist_pitch
                                // 左臂（7个）
                                0.0,   // left_shoulder_pitch
                                0.15,  // left_shoulder_roll
                                0.0,   // left_shoulder_yaw
                                1.4,   // left_elbow_pitch
                                0.0,   // left_wrist_yaw
                                0.0,   // left_wrist_pitch
                                0.0,   // left_wrist_roll
                                // 右臂（7个）
                                0.0,   // right_shoulder_pitch
                                -0.15, // right_shoulder_roll
                                0.0,   // right_shoulder_yaw
                                1.4,   // right_elbow_pitch
                                0.0,   // right_wrist_yaw
                                0.0,   // right_wrist_pitch
                                0.0,   // right_wrist_roll
                                // 左腿（6个）
                                -0.15,  // left_hip_pitch
                                0.0,   // left_hip_roll
                                0.0,   // left_hip_yaw
                                0.3,   // left_knee_pitch
                                -0.15,  // left_ankle_pitch
                                0.0,   // left_ankle_roll
                                // 右腿（6个）
                                -0.15,  // right_hip_pitch
                                0.0,   // right_hip_roll
                                0.0,   // right_hip_yaw
                                0.3,   // right_knee_pitch
                                -0.15,  // right_ankle_pitch
                                0.0;   // right_ankle_roll
        }
    }

    ~AmpZeroPosState() override {}

    void OnEnter() override {
        GetRobotJointValue();
        RecordJointData();
        motion_state_store_->SetState(RobotMotionState::AmpZeroPos);
    }

    void OnExit() override {}

    void Run() override {
        GetRobotJointValue();

        VecXf planning_joint_pos = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf planning_joint_vel = VecXf::Zero(cp_ptr_->dof_num_);
        VecXf planning_joint_tau = VecXf::Zero(cp_ptr_->dof_num_);

        float stand_duration_ = 3.;
        if (run_time_ - time_stamp_record_ <= stand_duration_) {
            for (int i = 0; i < current_joint_pos_.rows(); ++i) {
                planning_joint_pos(i) = GetCubicSplinePos(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0,
                                                      run_time_ - time_stamp_record_, stand_duration_);
                planning_joint_vel(i) = GetCubicSplineVel(init_joint_pos_(i), init_joint_vel_(i), goal_joint_pos_(i), 0,
                                                      run_time_ - time_stamp_record_, stand_duration_);
            }
        } else {
            for (int i = 0; i < current_joint_pos_.rows(); ++i) {
                planning_joint_pos(i) = goal_joint_pos_(i);
                planning_joint_vel.setZero();
            }
        }

        joint_cmd_.col(0) = kp_;
        joint_cmd_.col(2) = kd_;

        joint_cmd_.col(1) = planning_joint_pos;
        joint_cmd_.col(3) = planning_joint_vel;
        joint_cmd_.col(4) = planning_joint_tau;
        ri_ptr_->SetJointCommand(joint_cmd_);
        if (run_time_ - time_stamp_record_ >= stand_duration_)
            motion_state_store_->MarkHandsetReady();
    }

    bool LoseControlJudge() override {
        const auto command_snapshot = uc_ptr_->GetUserCommandSnapshot();
        if (command_snapshot.target_mode == RobotMotionState::JointDamping) {
            return true;
        }
        return false;
    }

    StateName GetNextStateName() override {
        const auto command_snapshot = uc_ptr_->GetUserCommandSnapshot();
        if (command_snapshot.safe_control_mode != 0 ||
            command_snapshot.target_mode == RobotMotionState::JointDamping) {
            std::cout << "safe_control_mode:" << std::dec << command_snapshot.safe_control_mode << std::endl;
            return StateName::kJointDamping;
        }
        // Finish the entry spline before handing control to another state.
        if (run_time_ - time_stamp_record_ < 3.0f) return StateName::kAmpZeroPos;


        if (command_snapshot.target_mode == RobotMotionState::AmpWholeBodyControlMode) {
            std::cout << "go to amp control" << std::endl;
            return StateName::kAmpWholeBodyControl;
        }

        if (command_snapshot.target_mode == RobotMotionState::AmpWalkingZeroPos) {
            std::cout << "go to amp walking zero pos" << std::endl;
            return StateName::kAmpWalkingZeroPos;
        }

        if (command_snapshot.target_mode == RobotMotionState::AmpControl_Walk_Mode) {
            std::cout << "go to amp walking control" << std::endl;
            return StateName::kAmpWalkingControl;
        }

        return StateName::kAmpZeroPos;
    }
};

}  // namespace humanoid_pro

#endif
