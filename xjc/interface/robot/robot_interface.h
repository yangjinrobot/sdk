/**
 * @file robot_interface.h
 * @brief this file is for applying robot's control interface
 * @author deeprobotics
 * @version 1.0
 * @date 2024-04-11
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */
#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_

#include "common_types.h"
#include "dds_types.h"
#include "neck_takeover.hpp"

#include <atomic>
#include <algorithm>
#include <cmath>

using namespace types;

namespace interface{
const int BATTERY_DATA_SIZE=8;

class RobotInterface{
private:
    /* data */
public:
    /**
     * @brief Construct a new Robot Interface object
     * @param  robot_name       robot name
     * @param  dof_num          The number of degrees of freedom of the robot.
     */
    RobotInterface(const std::string& robot_name, int dof_num, int leg_dof_num, int arm_dof_num, int waist_dof_num, int neck_dof_num):robot_name_(robot_name), dof_num_(dof_num),
                    leg_dof_num_(leg_dof_num),arm_dof_num_(arm_dof_num),waist_dof_num_(waist_dof_num),neck_dof_num_(neck_dof_num){
        start_flag_ = true;
        joint_cmd_ = Eigen::Matrix<float, Eigen::Dynamic, 5>::Zero(dof_num_, 5);
    }
    virtual ~RobotInterface(){};

    std::string robot_name_;
    const int dof_num_, leg_dof_num_, arm_dof_num_, waist_dof_num_, neck_dof_num_;
    Eigen::Matrix<float, Eigen::Dynamic, 5> joint_cmd_;
    std::atomic<bool> start_flag_;
    bool hstop_status = false;

    /**
     * @brief Start to get robot's state and send control command
     */
    virtual void Start() = 0;

    /**
     * @brief Stop to control the robot
     */
    virtual void Stop() = 0;

    /**
     * @brief Get the time stamp of the robot
     * @return double        time stamp
     */
    virtual double GetInterfaceTimeStamp() = 0;

    /**
     * @brief Get the joint position of the robot
     * @return VecXf        joint position vector
     */
    virtual VecXf GetJointPosition() = 0;
    virtual VecXf GetJointPositionRaw() = 0;

    /**
     * @brief Get the joint velocity of the robot
     * @return VecXf        joint velocity vector
     */
    virtual VecXf GetJointVelocity() = 0;

    /**
     * @brief Get the joint torque of the robot
     * @return VecXf        joint torque vector
     */
    virtual VecXf GetJointTorque() = 0;

    /**
     * @brief Get HeightMap(image)
     * @return VecXf hmap_vec
     */  
    virtual VecXf GetHeightMapVec() = 0;
    virtual bool GetHmapUpdateFlag() = 0;
    
    /**
     * @brief Get the roll-pitch-yaw angle of the robot base(waistz link),for leg
     * @return Vec3f        roll-pitch-yaw(unit rad)
     */
    virtual Vec3f GetImuRpy() = 0;

    /**
     * @brief Get the accleration of the robot base(waistz link),for leg
     * @return Vec3f        accleration(unit m*s^-2)
     */
    virtual Vec3f GetImuAcc() = 0;

    /**
     * @brief Get the angular velocity of robot base(waistz link),for leg
     * @return Vec3f        angular velocity in body coordinate(unit rad/s)
     */
    virtual Vec3f GetImuOmega() = 0;

    /**
     * @brief Get the roll-pitch-yaw angle of the robot body
     * @return Vec3f        roll-pitch-yaw(unit rad)
     */
    virtual Vec3f GetImuBodyRpy() = 0;

    /**
     * @brief Get the accleration of the robot body
     * @return Vec3f        accleration(unit m*s^-2)
     */
    virtual Vec3f GetImuBodyAcc() = 0;

    /**
     * @brief Get the angular velocity of robot body
     * @return Vec3f        angular velocity in body coordinate(unit rad/s)
     */
    virtual Vec3f GetImuBodyOmega() = 0;

    /**
     * @brief Set the joint command in standard form
     *                  torque = kp * (qDes - q) + kd * (vDes - v) + tff
     * @param  input       a dof_num*5 matrix and each column represent kp, goal_angle_pos, kd, goal_vel, torque_feedforward
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) = 0;
    virtual void SetJointZero(uint64_t jointMask) = 0;
    virtual void InitPositionOffset(uint64_t jointMask) = 0;
    // 仅执行360°换算修正（不含阻塞等待），标零流程专用
    virtual void WrapPositionOffset() {}

    virtual MatXf GetJointCommand(){
        return joint_cmd_;
    }

    // 脖子接管控制：转发到 utils::NeckTakeover。
    void SetNeckTakeoverEnabled(bool en) { neck_to_.SetEnabled(en); }
    bool IsNeckTakeoverEnabled() const   { return neck_to_.IsEnabled(); }
    void SetNeckTakeoverCmd(float yaw_cmd, float pitch_cmd) {
        neck_to_.SetCmd(yaw_cmd, pitch_cmd);
    }
    void SetNeckTakeoverParams(float yaw_range_rad, float pitch_range_rad,
                               float max_step_rad, float deadzone) {
        neck_to_.SetParams(yaw_range_rad, pitch_range_rad, max_step_rad, deadzone);
    }
    void LoadNeckTakeoverConfig(const std::string& path = "") {
        neck_to_.LoadFromToml(path, neck_dof_num_);
    }
    void ApplyNeckTakeover(Eigen::Matrix<float, Eigen::Dynamic, 5>& input) {
        // 仅在 neck 存在且接管启用时才实际拉取关节位置，避免不必要的
        // GetJointPosition() 开销；NeckTakeover::Apply 内部仍会二次 short-circuit。
        float yaw_actual = 0.f, pitch_actual = 0.f;
        if (neck_dof_num_ > 0 && neck_to_.IsEnabled()) {
            const VecXf q = GetJointPosition();
            if (q.size() >= dof_num_) {
                const int yaw_idx   = dof_num_ - neck_dof_num_;
                yaw_actual   = q(yaw_idx);
                if (neck_dof_num_ >= 2) pitch_actual = q(dof_num_ - 1);
            }
        }
        neck_to_.Apply(input, dof_num_, neck_dof_num_, yaw_actual, pitch_actual);
    }

protected:
    utils::NeckTakeover neck_to_;

public:

    /**
     * @brief Get the contact force if robot have any force sensor
     * @return VecXf        force vector
     */
    virtual VecXf GetContactForce() = 0;


    /**
     * @brief Get the motor temperture 
     * @return VecXf        motor temperture
     */
    virtual VecXf GetMotorTemperture(){
        // return VecXf::Zero(dof_num_);
        return 101.*VecXf::Ones(dof_num_);
    }

    /**
     * @brief Get the driver temperture 
     * @return VecXf        driver temperture
     */
    virtual VecXf GetDriverTemperture(){
        return VecXf::Zero(dof_num_);
    }

    /**
     * @brief Get the Battery Data
     * @return std::vector<uint16_t> level(0~100), voltage(mV), current(mA), cycles, temperture*4(C) 
     */
    virtual std::vector<uint16_t> GetBatteryData(){
        return std::vector<uint16_t>(BATTERY_DATA_SIZE, 0);
    }

    /**
     * @brief Get the base Imu Timestamp 
     * @return float 
     */
    virtual double GetImuTimestamp(){
        return 0.;
    }

    /**
     * @brief Get the body Imu Timestamp 
     * @return float 
     */
    virtual double GetImuBodyTimestamp(){
        return 0.;
    }
    /**
     * @brief Get the Driver Status Word 
     * @return std::vector<uint16_t> 
     */
    virtual std::vector<uint16_t> GetDriverStatusWord(){
        return std::vector<uint16_t>(dof_num_, 1);
    }

    /**
     * @brief Get the Joint Data ID object
     * @return std::vector<uint16_t> 
     */
    virtual std::vector<uint16_t> GetJointDataID(){
        return std::vector<uint16_t>(dof_num_, 0);
    }

    virtual void RefreshRobotData(){
    }
    virtual void ResetJointError(){
    }
    virtual BatteryInfo_dds* GetBatteryInfo() = 0;

     virtual void WriteSnapShot()
     {
    }
};


};//namespace interface

#endif
