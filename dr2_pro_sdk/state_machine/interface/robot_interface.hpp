/**
 * @file robot_interface.hpp
 * @brief this file is for applying robot's control interface
 * @author DeepRobotics
 * @version 1.0
 * @date 2025-12-12
 *
 * @copyright Copyright (c) 2025  DeepRobotics
 *
 */
#pragma once

#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cstring>
#include <functional>
#include <limits>
#include <mutex>
#include <time.h>

#include "basic_function.hpp"
#include "common_types.h"
#include "drdds/msg/imu_data.hpp"
#include "drdds/msg/joints.hpp"
#include "drdds/msg/joints_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace types;

namespace interface {
class RobotInterface {
protected:
    // Timing information
    double ri_ts_;   // Robot interface timestamp (seconds)
    double imu_leg_ts_, imu_body_ts_;  // IMU timestamp (milliseconds)

    // Sensor data
    Vec3f omega_body_;  // Angular velocity from IMU (rad/s)
    Vec3f rpy_body_;         // Roll-Pitch-Yaw from IMU (radians)
    Vec3f acc_body_;         // Linear acceleration from IMU (m/s²)

    Vec3f omega_leg_;  // Angular velocity from IMU (rad/s)
    Vec3f rpy_leg_;    // Roll-Pitch-Yaw from IMU (radians)
    Vec3f acc_leg_;    // Linear acceleration from IMU (m/s²)

    // Joint data
    VecXf joint_pos_;           // Joint positions (rad)
    VecXf joint_vel_;           // Joint velocities (rad/s)
    VecXf joint_tau_;           // Joint torques (Nm)
    VecXf motor_temperature_;   // Motor temperatures (°C)
    VecXf driver_temperature_;  // Driver temperatures (°C)

    // Configuration
    uint16_t* control_word = nullptr;  // Control words for joint reset sequence

    // Status monitoring
    std::vector<uint16_t> driver_status_;  // Joint driver status words
    std::vector<uint16_t> joint_data_id_;  // Joint data packet IDs

    rclcpp::Publisher<drdds::msg::JointsCmd>::SharedPtr joint_cmd_pub_;
    rclcpp::Subscription<drdds::msg::Joints>::SharedPtr joint_data_sub_;
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr imu_data_body_sub_;
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr imu_data_leg_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;

    std::mutex depth_mutex_;
    std::vector<float> depth_frame_;
    int depth_width_ = 0;
    int depth_height_ = 0;
    uint64_t depth_sequence_ = 0;

    int run_cnt_ = 0;  // Operation cycle counter

    // JOINTS_DATA transport latency statistics (receive time - message header timestamp)
    double joints_transport_latency_ms_ = 0.0;
    double joints_transport_latency_sum_ms_ = 0.0;
    double joints_transport_latency_min_ms_ = std::numeric_limits<double>::max();
    double joints_transport_latency_max_ms_ = 0.0;
    uint64_t joints_transport_latency_cnt_ = 0;

public:
    /**
     * @brief Construct a new Robot Interface object
     * @param  robot_name       robot name
     * @param  dof_num          The number of degrees of freedom of the robot.
     */
    RobotInterface(const std::string& robot_name, int dof_num, int leg_dof_num, int arm_dof_num, int waist_dof_num,
                   int neck_dof_num, rclcpp::Node::SharedPtr node)
        : robot_name_(robot_name),
          dof_num_(dof_num),
          leg_dof_num_(leg_dof_num),
          arm_dof_num_(arm_dof_num),
          waist_dof_num_(waist_dof_num),
          neck_dof_num_(neck_dof_num),
          joint_cmd_(Eigen::Matrix<float, Eigen::Dynamic, 5>::Zero(dof_num_, 5)),
          start_flag_(true),
          node_(node) {
        std::cout << robot_name << std::endl;
        std::cout << "Total DOF: " << dof_num_ << std::endl;
        std::cout << "  Leg DOF: " << leg_dof_num_ << std::endl;
        std::cout << "  Arm DOF: " << arm_dof_num_ << std::endl;
        std::cout << "Waist DOF: " << waist_dof_num_ << std::endl;
        std::cout << " Neck DOF: " << neck_dof_num_ << std::endl;

        if (!node_) {
            std::cerr << "RobotInterface node is null" << std::endl;
        }

        joint_pos_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        motor_temperature_ = VecXf::Zero(dof_num_);
        driver_temperature_ = VecXf::Zero(dof_num_);

        control_word = new uint16_t[3];

        driver_status_.resize(dof_num_);
        joint_data_id_.resize(dof_num_);

        control_word[0] = kIndexDisable;
        control_word[1] = kIndexErrorReset;
        control_word[2] = kIndexEnable;
    }

    virtual ~RobotInterface() {
        delete[] control_word;
    };

    std::string robot_name_;
    const int dof_num_, leg_dof_num_, arm_dof_num_, waist_dof_num_, neck_dof_num_;
    Eigen::Matrix<float, Eigen::Dynamic, 5> joint_cmd_;
    std::atomic<bool> start_flag_;
    rclcpp::Node::SharedPtr node_;

    /**
     * @brief Start to get robot's state and send control command
     */
    virtual void Start() {
        joint_cmd_pub_ = node_->create_publisher<drdds::msg::JointsCmd>("/JOINTS_CMD", 10);
        joint_data_sub_ = node_->create_subscription<drdds::msg::Joints>(
            "/JOINTS_DATA", 10, std::bind(&RobotInterface::Handler, this, std::placeholders::_1));
        imu_data_body_sub_ = node_->create_subscription<drdds::msg::ImuData>(
            "/IMU_DATA_BODY", 10, std::bind(&RobotInterface::HandlerIMUBody, this, std::placeholders::_1));
        imu_data_leg_sub_ = node_->create_subscription<drdds::msg::ImuData>(
            "/IMU_DATA_LEG", 10, std::bind(&RobotInterface::HandlerIMULeg, this, std::placeholders::_1));
        depth_image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "/DEPTH_FRAME", rclcpp::SensorDataQoS(),
            std::bind(&RobotInterface::HandlerDepthImage, this, std::placeholders::_1));
        WaitDataUpdate();
    }

    /**
     * @brief Stop to control the robot
     */
    virtual void Stop() {}

    /**
     * @brief Get the time stamp of the robot
     * @return double        time stamp
     */
    virtual double GetInterfaceTimeStamp() { return ri_ts_; }

    /**
     * @brief Get the joint position of the robot
     * @return VecXf        joint position vector
     */
    virtual VecXf GetJointPosition() { return joint_pos_; }

    /**
     * @brief Get the joint velocity of the robot
     * @return VecXf        joint velocity vector
     */
    virtual VecXf GetJointVelocity() { return joint_vel_; }

    /**
     * @brief Get the joint torque of the robot
     * @return VecXf        joint torque vector
     */
    virtual VecXf GetJointTorque() { return joint_tau_; }

    virtual Vec3f GetImuBodyRpy() { return rpy_body_; }

    virtual Vec3f GetImuBodyAcc() { return acc_body_; }

    virtual Vec3f GetImuBodyOmega() { return omega_body_; }

    /**
     * @brief Get the roll-pitch-yaw angle of the robot base(waistz link),for leg
     * @return Vec3f        roll-pitch-yaw(unit rad)
     */
    virtual Vec3f GetImuRpy() { return rpy_leg_; }

    /**
     * @brief Get the accleration of the robot base(waistz link),for leg
     * @return Vec3f        accleration(unit m*s^-2)
     */
    virtual Vec3f GetImuAcc() { return acc_leg_; }

    /**
     * @brief Get the angular velocity of robot base(waistz link),for leg
     * @return Vec3f        angular velocity in body coordinate(unit rad/s)
     */
    virtual Vec3f GetImuOmega() { return omega_leg_; }

    /**
     * @brief Set the joint command in standard form
     *                  torque = kp * (qDes - q) + kd * (vDes - v) + tff
     * @param  input       a dof_num*5 matrix and each column represent kp, goal_angle_pos, kd, goal_vel,
     * torque_feedforward
     */
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input) {
        joint_cmd_ = input;
        auto msg = drdds::msg::JointsCmd();
        msg.data.resize(dof_num_);

        for (int i = 0; i < dof_num_; ++i) {
            msg.data[i].position = input(i, 1);
            msg.data[i].velocity = input(i, 3);
            msg.data[i].torque = input(i, 4);
            msg.data[i].kp = input(i, 0);
            msg.data[i].kd = input(i, 2);
            msg.data[i].control_word = kIndexMotorControl;
        }

        joint_cmd_pub_->publish(msg);
    }

    virtual MatXf GetJointCommand() { return joint_cmd_; }

    /**
     * @brief Get the contact force if robot have any force sensor
     * @return VecXf        force vector
     */
    virtual VecXf GetContactForce() { return VecXf::Zero(4); }

    /**
     * @brief Get the motor temperture
     * @return VecXf        motor temperture
     */
    virtual VecXf GetMotorTemperature() { return motor_temperature_; }

    /**
     * @brief Get the driver temperture
     * @return VecXf        driver temperture
     */
    virtual VecXf GetDriverTemperature() { return driver_temperature_; }

    /**
     * @brief Get the base Imu Timestamp
     * @return float
     */
    virtual double GetImuTimestamp() { return imu_leg_ts_; }

    /**
     * @brief Get the body Imu Timestamp
     * @return float
     */
    virtual double GetImuBodyTimestamp() { return imu_body_ts_; }

    virtual double GetImuLegTimestamp() { return imu_leg_ts_; }

    virtual bool GetDepthFrame(std::vector<float>& frame, int& width, int& height,
                               uint64_t& sequence) {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        if (depth_frame_.empty()) {
            return false;
        }
        frame = depth_frame_;
        width = depth_width_;
        height = depth_height_;
        sequence = depth_sequence_;
        return true;
    }

    /**
     * @brief Get the Driver Status Word
     * @return std::vector<uint16_t>
     */
    virtual std::vector<uint16_t> GetDriverStatusWord() { return driver_status_; }

    /**
     * @brief Get the Joint Data ID object
     * @return std::vector<uint16_t>
     */
    virtual std::vector<uint16_t> GetJointDataID() { return joint_data_id_; }

    virtual void RefreshRobotData() {}

    virtual void WaitDataUpdate() {
        int cnt = 0;
        std::cout << "wait data update" << std::endl;
        while (rclcpp::ok()) {
            ++cnt;
            rclcpp::spin_some(node_);
            if (run_cnt_ > 0) {
                std::cout << "data updated at " << cnt << " cnt!" << std::endl;
                break;
            }
            usleep(1000);
            if (cnt == 10000) {
                std::cout << "joint data update is not finished\n";
            }
        }
    }

    virtual void Handler(const drdds::msg::Joints::SharedPtr msg) {
        const double recv_monotonic_ts_s = GetTimestampMs() / 1000.;
        timespec recv_realtime;
        clock_gettime(CLOCK_REALTIME, &recv_realtime);
        const double recv_realtime_ts_s =
            static_cast<double>(recv_realtime.tv_sec) + static_cast<double>(recv_realtime.tv_nsec) * 1e-9;
        const auto& stamp = msg->header.stamp;
        const double msg_ts_s = static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
        joints_transport_latency_ms_ = (recv_realtime_ts_s - msg_ts_s) * 1e3;
        joints_transport_latency_sum_ms_ += joints_transport_latency_ms_;
        ++joints_transport_latency_cnt_;
        joints_transport_latency_min_ms_ = std::min(joints_transport_latency_min_ms_, joints_transport_latency_ms_);
        joints_transport_latency_max_ms_ = std::max(joints_transport_latency_max_ms_, joints_transport_latency_ms_);

        if (joints_transport_latency_cnt_ % 1000 == 0) {
            std::cout << "JOINTS_DATA transport latency(ms): cur=" << joints_transport_latency_ms_
                      << ", avg=" << joints_transport_latency_sum_ms_ / joints_transport_latency_cnt_
                      << ", min=" << joints_transport_latency_min_ms_
                      << ", max=" << joints_transport_latency_max_ms_ << std::endl;
        }

        ++run_cnt_;
        for (int i = 0; i < dof_num_; ++i) {
            joint_pos_(i) = msg->data[i].position;
            joint_vel_(i) = msg->data[i].velocity;
            joint_tau_(i) = msg->data[i].torque;
            motor_temperature_(i) = float(msg->data[i].motion_temp);
            driver_temperature_(i) = float(msg->data[i].driver_temp);
            driver_status_[i] = msg->data[i].status_word;
            joint_data_id_[i] = uint16_t(run_cnt_);
        }

        ri_ts_ = recv_monotonic_ts_s;
    }

    virtual void HandlerDepthImage(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (msg->encoding != "32FC1" || msg->width == 0 || msg->height == 0 ||
            msg->step < msg->width * sizeof(float) ||
            msg->data.size() < static_cast<size_t>(msg->step) * msg->height) {
            return;
        }

        std::vector<float> frame(static_cast<size_t>(msg->width) * msg->height);
        for (uint32_t row = 0; row < msg->height; ++row) {
            const auto* src = reinterpret_cast<const float*>(msg->data.data() + row * msg->step);
            std::copy(src, src + msg->width, frame.begin() + static_cast<size_t>(row) * msg->width);
        }

        std::lock_guard<std::mutex> lock(depth_mutex_);
        depth_frame_ = std::move(frame);
        depth_width_ = static_cast<int>(msg->width);
        depth_height_ = static_cast<int>(msg->height);
        ++depth_sequence_;
    }

    virtual void HandlerIMUBody(const drdds::msg::ImuData::SharedPtr msg) {
        rpy_body_ = Vec3f(Deg2Rad(msg->data.roll), Deg2Rad(msg->data.pitch), Deg2Rad(msg->data.yaw));
        acc_body_ << msg->data.acc_x, msg->data.acc_y, msg->data.acc_z;
        omega_body_ << msg->data.omega_x, msg->data.omega_y, msg->data.omega_z;
        imu_body_ts_ = GetTimestampMs();
    }

    virtual void HandlerIMULeg(const drdds::msg::ImuData::SharedPtr msg) {
        rpy_leg_ = Vec3f(Deg2Rad(msg->data.roll), Deg2Rad(msg->data.pitch), Deg2Rad(msg->data.yaw));
        acc_leg_ << msg->data.acc_x, msg->data.acc_y, msg->data.acc_z;
        omega_leg_ << msg->data.omega_x, msg->data.omega_y, msg->data.omega_z;
        imu_leg_ts_ = GetTimestampMs();
    }

};

};  // namespace interface
