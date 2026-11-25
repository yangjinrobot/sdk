#include "humanoid_pro/amp_walking_zeropos_state.hpp"

#include <iostream>

namespace humanoid {

AmpWalkingZeroPosState::AmpWalkingZeroPosState(const RobotName& robot_name, const std::string& state_name,
                                               std::shared_ptr<ControllerData> data_ptr)
    : StateBase(robot_name, state_name, data_ptr) {
    joint_pos_goal_ = VecXf::Zero(cp_ptr_->dof_num_);
    joint_cmd_ = MatXf::Zero(cp_ptr_->dof_num_, 5);
    kp_ = VecXf::Zero(cp_ptr_->dof_num_);
    kd_ = VecXf::Zero(cp_ptr_->dof_num_);

    if (robot_name_ == RobotName::CR1PRO) {
        kp_ << cp_ptr_->waist_kp_, cp_ptr_->arm_kp_, cp_ptr_->arm_kp_, cp_ptr_->leg_kp_, cp_ptr_->leg_kp_,
            cp_ptr_->neck_kp_;
        kd_ << cp_ptr_->waist_kd_, cp_ptr_->arm_kd_, cp_ptr_->arm_kd_, cp_ptr_->leg_kd_, cp_ptr_->leg_kd_,
            cp_ptr_->neck_kd_;

        // 设置默认关节目标位置（29个关节，顺序与控制器内部关节顺序一致）
        joint_pos_goal_.head(29) << 0.0, // waist_yaw
            0.0,                         // waist_roll
            0.0,                         // waist_pitch

            // 左臂（7个）
            0.0,  // left_shoulder_pitch
            0.15, // left_shoulder_roll
            0.0,  // left_shoulder_yaw
            1.4,  // left_elbow_pitch
            0.0,  // left_wrist_yaw
            0.0,  // left_wrist_pitch
            0.0,  // left_wrist_roll

            // 右臂（7个）
            0.0,   // right_shoulder_pitch
            -0.15, // right_shoulder_roll
            0.0,   // right_shoulder_yaw
            1.4,   // right_elbow_pitch
            0.0,   // right_wrist_yaw
            0.0,   // right_wrist_pitch
            0.0,   // right_wrist_roll

            // 左腿（6个）
            -0.15, // left_hip_pitch
            0.0,   // left_hip_roll
            0.0,   // left_hip_yaw
            0.3,   // left_knee_pitch
            -0.15, // left_ankle_pitch
            0.0,   // left_ankle_roll

            // 右腿（6个）
            -0.15, // right_hip_pitch
            0.0,   // right_hip_roll
            0.0,   // right_hip_yaw
            0.3,   // right_knee_pitch
            -0.15, // right_ankle_pitch
            0.0;   // right_ankle_roll
    }
}

AmpWalkingZeroPosState::~AmpWalkingZeroPosState() = default;

void AmpWalkingZeroPosState::OnEnter() {
    GetRobotJointValue();
    RecordJointData();
    print_counter_ = 0;
    motion_state_store_->SetState(RobotMotionState::AmpWalkingZeroPos);
}

void AmpWalkingZeroPosState::OnExit() {}

void AmpWalkingZeroPosState::Run() {
    GetRobotJointValue();

    VecXf joint_pos_plan = VecXf::Zero(cp_ptr_->dof_num_);
    VecXf joint_vel_plan = VecXf::Zero(cp_ptr_->dof_num_);
    VecXf joint_tor_plan = VecXf::Zero(cp_ptr_->dof_num_);

    float init_time = 3.;
    if (run_time_ - state_enter_time_ <= init_time) {
        for (int i = 0; i < joint_pos_.rows(); ++i) {
            joint_pos_plan(i) = GetCubicSplinePos(joint_pos_init_(i), joint_vel_init_(i), joint_pos_goal_(i),
                                                  0, run_time_ - state_enter_time_, init_time);
            joint_vel_plan(i) = GetCubicSplineVel(joint_pos_init_(i), joint_vel_init_(i), joint_pos_goal_(i),
                                                  0, run_time_ - state_enter_time_, init_time);
        }
    } else {
        for (int i = 0; i < joint_pos_.rows(); ++i) {
            joint_pos_plan(i) = joint_pos_goal_(i);
            joint_vel_plan.setZero();
        }
    }

    joint_cmd_.col(0) = kp_;
    joint_cmd_.col(2) = kd_;

    joint_cmd_.col(1) = joint_pos_plan;
    joint_cmd_.col(3) = joint_vel_plan;
    joint_cmd_.col(4) = joint_tor_plan;
    ri_ptr_->SetJointCommand(joint_cmd_);

    if (++print_counter_ >= 500) {
        print_counter_ = 0;
        static const double RAD_TO_DEG = 180.0 / 3.14159265358979323846;
        static const std::vector<std::string> joint_names = {
            "waist_yaw",          "waist_roll",        "waist_pitch",          "left_shoulder_pitch",
            "left_shoulder_roll", "left_shoulder_yaw", "left_elbow_pitch",     "left_wrist_yaw",
            "left_wrist_pitch",   "left_wrist_roll",   "right_shoulder_pitch", "right_shoulder_roll",
            "right_shoulder_yaw", "right_elbow_pitch", "right_wrist_yaw",      "right_wrist_pitch",
            "right_wrist_roll",   "left_hip_pitch",    "left_hip_roll",        "left_hip_yaw",
            "left_knee_pitch",    "left_ankle_pitch",  "left_ankle_roll",      "right_hip_pitch",
            "right_hip_roll",     "right_hip_yaw",     "right_knee_pitch",     "right_ankle_pitch",
            "right_ankle_roll"};

        std::cout << "[AmpWalkingZeroPos] current/target joint positions (deg): ";
        for (int i = 0; i < joint_pos_.rows() && i < static_cast<int>(joint_names.size()); ++i) {
            std::cout << joint_names[i] << ":" << joint_pos_(i) * RAD_TO_DEG << "/"
                      << joint_pos_plan(i) * RAD_TO_DEG << " ";
        }
        std::cout << std::endl;
    }
}

bool AmpWalkingZeroPosState::LoseControlJudge() {
    const auto command_snapshot = uc_ptr_->GetUserCommandSnapshot();
    if (command_snapshot.target_mode == RobotMotionState::JointDamping) {
        return true;
    }
    return false;
}

StateName AmpWalkingZeroPosState::GetNextStateName() {
    const auto command_snapshot = uc_ptr_->GetUserCommandSnapshot();
    if (command_snapshot.safe_control_mode != 0 ||
        command_snapshot.target_mode == RobotMotionState::JointDamping) {
        std::cout << "safe_control_mode:" << std::dec << command_snapshot.safe_control_mode << std::endl;
        return StateName::kJointDamping;
    }
    // Finish the entry spline before handing control to another state.
    if (run_time_ - state_enter_time_ < 3.0f)
        return StateName::kAmpWalkingZeroPos;

    if (command_snapshot.target_mode == RobotMotionState::AmpWalkingControl) {
        std::cout << "go to amp control" << std::endl;
        return StateName::kAmpWalkingControl;
    }

    if (command_snapshot.target_mode == RobotMotionState::AmpWalkingWAQ)
        return StateName::kAmpWalkingWAQ;
    return StateName::kAmpWalkingZeroPos;
}

void AmpWalkingZeroPosState::GetRobotJointValue() {
    run_time_ = ri_ptr_->GetInterfaceTimeStamp();
    joint_pos_ = ri_ptr_->GetJointPosition();
    joint_vel_ = ri_ptr_->GetJointVelocity();
}

void AmpWalkingZeroPosState::RecordJointData() {
    state_enter_time_ = run_time_;
    joint_pos_init_ = joint_pos_;
    joint_vel_init_ = joint_vel_;
}

} // namespace humanoid
