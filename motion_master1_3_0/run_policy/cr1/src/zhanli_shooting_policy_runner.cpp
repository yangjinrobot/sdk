#include "zhanli_shooting_policy_runner.hpp"

#include <stdexcept>

ZhanliShootingPolicyRunner::ZhanliShootingPolicyRunner(
    const std::string& policy_name, RobotName robot_name,
    const std::string& onnx_path)
    : CommonPolicyRunner(policy_name, robot_name, onnx_path,
                         GraphOptimizationLevel::ORT_DISABLE_ALL) {
    if (robot_name != RobotName::CR1STANDARD) {
        throw std::runtime_error(
            "policy_shoot_walk.onnx supports CR1STANDARD (21 DOF) only");
    }

    // distinct from CommonPolicyRunner's walking-policy default pose.
    dof_pos_default_ << -0.2f, -0.2f, 0.f, 0.f, 0.f, 0.f, 0.f,
        0.f, 0.f, 0.15f, -0.15f, 0.4f, 0.4f, 0.f, 0.f,
        -0.2f, -0.2f, 1.2f, -0.2f, 0.f, 0.f;

    // PD gains from shooting_0916 training, in POLICY_JOINT_ORDER.
    VecXf kp(21);


    kp << 300.0, 300.0,  // left_hip_y_joint, right_hip_y_joint
            200.0,            // waist_z_joint
            300.0, 300.0,    // left_hip_x_joint, right_hip_x_joint
            100.0, 100.0,    // left_shoulder_y_joint, right_shoulder_y_joint
            300.0, 300.0,    // left_hip_z_joint, right_hip_z_joint
            100.0, 100.0,    // left_shoulder_x_joint, right_shoulder_x_joint
            300.0, 300.0,    // left_knee_joint, right_knee_joint
            100.0, 100.0,    // left_shoulder_z_joint, right_shoulder_z_joint
            80.0, 80.0,      // left_ankle_y_joint, right_ankle_y_joint
            100.0, 100.0,    // left_elbow_joint, right_elbow_joint
            30.0, 30.0;      // left_ankle_x_joint, right_ankle_x_joint
    kp = kp * 0.9;
    
    VecXf kd(21);
    kd << 10.f, 10.f, 5.f, 10.f, 10.f, 5.f, 5.f,
        5.f, 5.f, 5.f, 5.f, 10.f, 10.f, 5.f, 5.f,
        3.f, 3.f, 5.f, 5.f, 1.f, 1.f;
    SetPdGains(kp, kd);
}
