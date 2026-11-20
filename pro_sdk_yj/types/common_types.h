
#ifndef COMMON_TYPES_H_
#define COMMON_TYPES_H_

#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include <vector>
#include <deque>
#include <map>
#include <cmath>
#include <memory>
#include <thread>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <stdio.h>
#include <csignal>


namespace types{
    using Vec2f = Eigen::Vector2f;
    using Vec3f = Eigen::Vector3f;
    using Vec3d = Eigen::Vector3d;
    using Vec4f = Eigen::Vector4f;
    using Vec4d = Eigen::Vector4d;
    using Vec6f = Eigen::Matrix<float, 6, 1>;
    using Vec6d = Eigen::Matrix<double, 6, 1>;
    using Vec7f = Eigen::Matrix<float, 7, 1>;
    using Vec7d = Eigen::Matrix<double, 7, 1>;
    using VecXf = Eigen::VectorXf;
    using VecXd = Eigen::VectorXd;

    using Mat3f = Eigen::Matrix3f;
    using Mat3d = Eigen::Matrix3d;
    using MatXf = Eigen::MatrixXf;
    using MatXd = Eigen::MatrixXd;

    const float gravity = 9.815;
    struct RobotBasicState{
        Vec3f base_rpy;
        Vec4f base_quat;
        Mat3f base_rot_mat;
        Vec3f base_omega;
//        Vec3f flt_base_omega;
        MatXf flt_base_omega_mat;
        Vec3f base_acc;
//        Vec3f flt_base_acc;
        MatXf flt_base_acc_mat;
        VecXf joint_pos;
        VecXf joint_vel;
//        VecXf flt_joint_vel;
        MatXf flt_joint_vel_mat;
        VecXf joint_tau;
        VecXf hmap_vec;
        int motion_idx = 0;
};

    struct RobotAction{
        VecXf goal_joint_pos;
        VecXf goal_joint_vel;
        VecXf kp;
        VecXf kd;
        VecXf tau_ff;

        MatXf ConvertToMat(){
            MatXf res(goal_joint_pos.rows(), 5);
            res.col(0) = kp; res.col(1) = goal_joint_pos;
            res.col(2) = kd; res.col(3) = goal_joint_vel;
            res.col(4) = tau_ff;
            return res;
        }
    };
    
    typedef enum
    {
        kSafeDriverStatus = 0,
        kSafeJointUpdate = 1,
        kSafePostureNormal = 2,
        kSafeImuData = 3,
        kSafeMotorTemp = 4,
        kSafeDriverTemp = 5,
        kSafeJointNan = 6,
        kSafeJointPosLimit = 7,
        kSafeJointVelLimit = 8,

        kSafeEnd = 31,
    }SAFE_CONTROL_MODE_OFFSET;
    typedef enum
    {
        kWarnBattery = 0,
        kWarnMotorTemp = 1,
        kWarnDriverTemp = 2,

        kWarnEnd = 31,
    }ROBOT_WARN_MODE_OFFSET;
    struct UserCommand{
        double time_stamp;
        uint32_t safe_control_mode; 
        uint32_t target_mode;
        uint32_t target_gait;
        uint32_t target_action;
        float forward_vel_scale;
        float side_vel_scale;
        float turnning_vel_scale;
        float targetPitch=0;
        float targetRoll=0;
        float targetHeight=0;
        bool arm_preset_1 = false;
        bool arm_preset_2 = false;
        bool arm_preset_3 = false;
        bool arm_preset_4 = false;
        bool arm_preset_5 = false;
        bool arm_preset_6 = false;
        int arm_preset = 0;
        uint32_t arm_preset_seq = 0;
        bool disable_arm_motion_planning = false;
        // 脖子接管指令归一化值 [-1.0, 1.0]，仅在 neck_takeover_control 启用后被消费。
        float neck_yaw_cmd=0;    // → neckyaw
        float neck_pitch_cmd=0;  // → neckpitch
    };
};

#endif