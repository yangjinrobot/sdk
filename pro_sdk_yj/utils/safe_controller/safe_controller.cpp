/**
 * @file safe_controller.cpp
 * @brief safe controller to make robot safe
 * @author deeprobotics
 * @version 1.0
 * @date 2024-08-15
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

/**
 * 一、关节端
 *      1.关节数据异常（nan值）
 *      2.关节数据不更新
 *      3.关节温度过温
 *      4.驱动器报错
 * 二、imu
 *      1.imu数据异常
 *      2.imu数据不更新
 * 三、电池
 *      1.电池电量过低
 *      2.电池数据不上报？
 *      3.电池温度？
 * 四、手柄
 *      1.手柄断连
 *      
 */



#include "safe_controller/safe_controller.hpp"
#include "state_base.hpp"
#include "toml/toml.hpp"
#include "runtime_params.hpp"
#include "robot_hwi.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>

namespace {

// 依赖关节观测值(pos/vel/tau)或与之同源的关节故障码；关节数据不更新时这些检测结果不可信，
// 应抑制上报（软急停仍由各检测函数照常触发）。新增关节类故障码只需在此登记。
bool IsJointDataDependentFault(int code) {
    switch (code) {
        case JNT_DATA_NAN:      case JNT_POS_OVER_LIM:   case JNT_VEL_OVER_LIM:
        case JNT_DRV_OVER_VOLT: case JNT_DRV_UNDER_VOLT: case JNT_DRV_OVER_CURR:
        case JNT_DRV_TIMEOUT:   case TEMP_SENSOR_DISCON:  case ENC_CALIB_FAIL:
        case JNT_ENC_NO_VAL:    case JNT_DRV_STATUS_ERR:
        case MOTOR_OVER_TEMP:   case JNT_DRV_OVER_TEMP:
            return true;
        default:  // JNT_DATA_UPD_ERR / IMU_TIMEOUT / IMU_NAN / BODY_POSE_ERR 不抑制
            return false;
    }
}  // namespace

}  // namespace

/*
 * @brief 报告故障状态变化（仅在资源位图或严重程度变化时发布）
 * @param current_resources 当前资源位图（0 表示故障恢复）
 * @param fault_code 故障代码
 * @param severity 每个故障资源对应的严重程度向量
 * @param grouped 是否分组上报
 * @param resource 当 grouped==false 时的单资源标识
 */
void SafeController::reportFault(
    uint32_t current_resources,
    int fault_code,
    const std::vector<uint8_t>& severity,
    bool grouped,
    const std::string& resource)
{
    /* judge_thread_ 与 ecan 接收线程（CheckMotorCommStatus/CheckMainboardVoltage）都会
     * 调用本函数，last_reported_state_ 与 pubEvent 需要互斥保护 */
    std::lock_guard<std::mutex> fault_lock(fault_report_mutex_);

    // 关节数据不更新时，抑制依赖关节观测数据的关节类故障上报（软急停由各检测函数照常触发）
    if (joint_data_not_updated_ && IsJointDataDependentFault(fault_code)) {
        return;
    }
    // Check if state changed compared to last report for this fault_code
    auto& last = last_reported_state_[fault_code];
    if (current_resources == last.resources && severity == last.severities) {
        return;  // No change, skip publishing
    }

    drdds::msg::FaultEvent fault_msg;
    fault_msg.code() = fault_code;
    fault_msg.severities() = severity;
    fault_msg.grouped() = grouped;

    if (current_resources == 0) {
        fault_msg.type() = FaultType::STOP;
    } else if (last.resources == 0) {
        fault_msg.type() = FaultType::START;
    } else {
        fault_msg.type() = FaultType::START;  // Update existing fault (was PULSE)
    }

    if (grouped) {
        fault_msg.resources() = getSetBitsVector(current_resources);
    } else {
        fault_msg.resources() = std::vector<std::string>{resource};
    }

    int32_t ret = fault_client_->pubEvent(fault_msg);
    if (ret != 0) {
        safe_logger_.error("pubEvent failed: code=0x%x ret=%d", fault_code, ret);
    } else {
        const char* type_str = (current_resources == 0) ? "STOP" : (last.resources == 0 ? "START" : "UPDATE");
        safe_logger_.info("reportFault: code=0x%x type=%s resources=0x%x sev_cnt=%zu",
                         fault_code, type_str, current_resources, severity.size());
    }

    // Update last reported state
    last.resources = current_resources;
    last.severities = severity;
}
    /**
 * @brief 将uint32_t转换为表示置1位位置的字符串向量
 * @param value 32位无符号整数
 * @return 包含所有置1位位置字符串的向量
 */
std::vector<std::string> SafeController::getSetBitsVector(uint32_t value) {
    std::vector<std::string> bit_positions;
    
    if (value == 0) {
        bit_positions.push_back("none");
        return bit_positions;
    }
    
    // 遍历32位
    for (int i = 0; i < 32; ++i) {
        if (value & (1u << i)) {
            bit_positions.push_back(std::to_string(i));
        }
    }
    
    return bit_positions;
}

    Vec3f SafeController::GroundContactDetection(Vec3f pos, Vec3f velocity, Vec3f torque)
    {
        float d1 = 0.2; float d2 = 0.2;
        float m1 = 2.852703+1.151106; float m2 = 4.249883;
        float I0 = 0.2; float I0_comp = 0.0;
        float I1 = 0.2; float I1_comp = 0.0;
        float I2 = 0.1; float I2_comp = 0.0;

        float l0 = 0.162;  //distance between joints
        float l1 = 0.420;
        float l2 = 0.410;
        float s0 = pos[0];  // joint angle
        float s1 = pos[1];
        float s2 = pos[2];
        float v0 = velocity[0];  // joint angle velocity (current cycle time)
        float v1 = velocity[1];
        float v2 = velocity[2];
        float kGravity = 9.81;

        Vec3f pseudo_torque;
        Vec3f actual_torque; 
        Vec3f delta_torque;
        Vec3f estimated_force_tmp;
        Vec3f estimated_force; 

        pseudo_torque[0] = 0;

        pseudo_torque[1] = -2 * m2*l1*d2*v1*v2 - m2 * l1*d2*v2*v2*sin(s2)
            + m1 * kGravity*d1*sin(s1) + m2 * kGravity*l1*sin(s1) + m2 * kGravity*d2*sin(s1 + s2);
        pseudo_torque[2] = v1 * v2*(m2*l1*d2*sin(s2) + m2 * l1*d2*sin(s2)) + v1 * v1*m2*l1*d2*sin(s2)
            + m2 * kGravity*d2*sin(s1 + s2);

        actual_torque[0] = torque[0];
        actual_torque[1] = torque[1];
        actual_torque[2] = torque[2];

        delta_torque[0] = actual_torque[0] - pseudo_torque[0];
        delta_torque[1] = actual_torque[1] - pseudo_torque[1];
        delta_torque[2] = actual_torque[2] - pseudo_torque[2];

        estimated_force_tmp[0] = ((l2*sin(s2 + s1) + l1 * sin(s1))*delta_torque[2] - l2 * sin(s2 + s1)*delta_torque[1]) / (l2*sin(s2 + s1)*l1*cos(s1) - l2 * cos(s1 + s2)*l1*sin(s1) + 0.00000001);
        estimated_force_tmp[2] = (l2*cos(s2 + s1)*delta_torque[1] - (l2*cos(s2 + s1) + l1 * cos(s1))*delta_torque[2]) / (l2*sin(s2 + s1)*l1*cos(s1) - l2 * cos(s1 + s2)*l1*sin(s1) + 0.00000001);
        estimated_force_tmp[1] = (estimated_force_tmp[2] * l0 + delta_torque[0]) / (l1*cos(s1) + l2 * cos(s1 + s2));

    // float J11 = -l1 * sinf(s1) - l2 * sinf(s1 + s2);
    // float J12 = -l2 * sinf(s1 + s2);
    // float J21 = l1 * cosf(s1) + l2 * cosf(s1 + s2);
    // float J22 = l2 * cosf(s1 + s2);

    // float det_JT = J11 * J22 - J12 * J21;

    // const float EPS = 1e-6f;
    // if (fabsf(det_JT) < EPS) { 
    //     throw std::runtime_error("雅可比矩阵奇异（二连杆处于完全伸直/折叠位形），无法计算接触力！");
    // }

    // float inv_JT_00 = J22 / det_JT;
    // float inv_JT_01 = -J21 / det_JT;
    // float inv_JT_10 = -J12 / det_JT;
    // float inv_JT_11 = J11 / det_JT;

    // float Fx = inv_JT_00 * delta_torque[1] + inv_JT_01 * delta_torque[2];
    // float Fy = inv_JT_10 * delta_torque[1] + inv_JT_11 * delta_torque[2];
    // std::cout<<"Fx Fy:"<<Fx<<", "<<Fy<<std::endl;

        //坐标系转换：关节系→机体系
        // Vec3f tmp;
        // Vec3f body_rpy;
        // Mat3f f_trans;
        // Vec3f rpy = ri_ptr_->GetImuRpy();

        // body_rpy << rpy[0], rpy[1], 0.0;
        // tmp << -s0, 0, 0;
        // f_trans = RolPitYawToRM(tmp).transpose() * RolPitYawToRM(body_rpy, 0).transpose(); 

        // estimated_force = f_trans.inverse()* estimated_force_tmp;

        return estimated_force_tmp;
    }

    bool SafeController::IsImuDataNormal(){
        double current_imu_ts_ = ri_ptr_->GetImuTimestamp();
        Vec3f rpy = ri_ptr_->GetImuRpy();
        Vec3f acc = ri_ptr_->GetImuAcc();
        Vec3f omg = ri_ptr_->GetImuOmega();
        bool res = true;
        std::vector<uint8_t> severity = {0};
        if(imu_check_flag_){
            last_imu_ts_ = current_imu_ts_;
            imu_check_time_ = current_time_;
            imu_check_flag_ = false;
            last_rpy = rpy;
            last_acc = acc;
            last_omg = omg;
            imu_data_same_cnt_ = 0;
        }

        // imu数据更新超时检测 40ms
        robot_error_state_.imu_update_overtime = 0;
        if(current_imu_ts_ != last_imu_ts_){
            last_imu_ts_ = current_imu_ts_;
            imu_check_time_ = current_time_;
        }
        bool timeout_cond = (current_time_ - imu_check_time_ > imu_update_timeout_threshold_);

        // imu数据相同检测：9个分量全部与上一帧相等才算"卡死"
        bool isImuDataSame = true;
        for(int i=0;i<3;++i){
            if(rpy(i) != last_rpy(i) || acc(i) != last_acc(i) || omg(i) != last_omg(i)){
                isImuDataSame = false;
                break;
            }
        }
        // 无论是否相同，均更新基线为当前帧，下一帧与"上一帧"比较
        last_rpy = rpy;
        last_acc = acc;
        last_omg = omg;
        if(isImuDataSame){
            imu_data_same_cnt_++;
        }else{
            imu_data_same_cnt_ = 0;
        }
        bool stuck_cond = (imu_data_same_cnt_ > imu_update_timeout_threshold_);

        // 延迟确认：连续 kImuTimeoutConfirmFrames 帧超时才置位，连续 kImuNormalConfirmFrames 帧正常才清零
        // 避免单帧瞬时超时（如 DDS 短暂抖动）触发 START 并联动停机
        if (timeout_cond || stuck_cond) {
            imu_timeout_cnt_++;
            imu_normal_cnt_ = 0;
        } else {
            imu_normal_cnt_++;
            imu_timeout_cnt_ = 0;
        }

        // 合并上报：两路径共用同一个flag，统一赋值，避免来回踩踏导致start/stop抖动
        if (imu_timeout_cnt_ >= kImuTimeoutConfirmFrames) {
            robot_error_state_.imu_update_overtime = 1;
            res = false;
            imu_update_timeout_flag_.store(1, std::memory_order_release);
            severity[0] = imu_timeout_fault_severity_;
        } else if (imu_normal_cnt_ >= kImuNormalConfirmFrames) {
            imu_update_timeout_flag_.store(0, std::memory_order_release);
            severity[0] = 0;
        }
        // imu_update_timeout_flag_ 保持上一次状态（计数期间不变），避免单帧抖动

        // 计数期间不调用 reportFault，仅在确认态变化时上报
        if (report_imu_data_timeout_ &&
            (imu_timeout_cnt_ >= kImuTimeoutConfirmFrames ||
             imu_normal_cnt_ >= kImuNormalConfirmFrames)) {
            reportFault(imu_update_timeout_flag_.load(std::memory_order_relaxed), 
                            IMU_TIMEOUT,
                            severity,
                            false,
                            "imu");
        }
        
        severity[0] = 0;
        // imu数据nan检测：任一轴分量为NaN即触发，全正常才清零
        robot_error_state_.imu_num_error = 0;
        bool any_nan = false;
        for(int i=0;i<3;++i){
            if(std::isnan(rpy(i)) || std::isnan(acc(i)) || std::isnan(omg(i))){
                res = false;
                any_nan = true;
            }
        }
        if (any_nan) {
            robot_error_state_.imu_num_error = 1;
            imu_data_nan_flag_.store(1, std::memory_order_release);
            severity[0] = imu_nan_fault_severity_;
        } else {
            imu_data_nan_flag_.store(0, std::memory_order_release);
        }
        if (report_imu_data_nan_) {
            reportFault(imu_data_nan_flag_.load(std::memory_order_relaxed), 
                            IMU_NAN,
                            severity,
                            false,
                            "imu");
        }

        return res;
    }
    // ── IsJointDataUpdateNormal / IsJointDataNan / IsJointPosLimitNormal /
    //     IsJointVelLimitNormal 已迁移至 JointMonitor ──

    bool SafeController::IsGroundContactNormal(){
        if(ri_ptr_->hstop_status)        return true;

        bool res = true;

        l_leg_joint_pos_<<joint_pos[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+1],joint_pos[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_],joint_pos[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+3];//xyk
        r_leg_joint_pos_<<joint_pos[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+7],joint_pos[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+6],joint_pos[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+9];//xyk
        l_leg_joint_vel_<<joint_vel[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+1],joint_vel[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_],joint_vel[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+3];//xyk
        r_leg_joint_vel_<<joint_vel[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+7],joint_vel[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+6],joint_vel[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+9];//xyk
        l_leg_joint_tau_<<joint_tau[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+1],joint_tau[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_],joint_tau[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+3];//xyk
        r_leg_joint_tau_<<joint_tau[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+7],joint_tau[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+6],joint_tau[2*cp_ptr_->arm_dof_num_+cp_ptr_->waist_dof_num_+9];//xyk
        Vec3f l_leg_estimated_force = GroundContactDetection(l_leg_joint_pos_,l_leg_joint_vel_,l_leg_joint_tau_);
        Vec3f r_leg_estimated_force = GroundContactDetection(r_leg_joint_pos_,r_leg_joint_vel_,r_leg_joint_tau_);

        if(l_leg_estimated_force[2]<100) l_leg_leave_grd_init_time_ = current_time_;
        if(r_leg_estimated_force[2]<100) r_leg_leave_grd_init_time_ = current_time_;
        if(current_time_ - l_leg_leave_grd_init_time_ > 0.2) l_leg_contact_state_ = 1;
        else l_leg_contact_state_ = 0;
        if(current_time_ - r_leg_leave_grd_init_time_ > 0.2) r_leg_contact_state_ = 1;
        else r_leg_contact_state_ = 0;
        // NOTE: 着地检测暂时禁用，强制设为接触状态。
        // GroundContactDetection 在部分工况下误判率高，预先屏蔽 BODY_POSE_ERR 着地路径，
        // 姿态异常检测仍由 CheckPostureLimit() (roll/pitch 阈值) 独立覆盖。
        l_leg_contact_state_ = 1;
        r_leg_contact_state_ = 1;
        // if(run_cnt_ % 2000 == 0){
        //     std::cout<<"l_leg_estimated_force:"<<l_leg_estimated_force.transpose()<<std::endl;
        //     std::cout<<"r_leg_estimated_force:"<<r_leg_estimated_force.transpose()<<std::endl;
        //     std::cout<<"l_leg_contact_state_:"<<l_leg_contact_state_<<std::endl;
        //     std::cout<<"l_leg_contact_state_:"<<r_leg_contact_state_<<std::endl;
        //     std::cout<<"-------------------"<<std::endl;
        // }
       
        
        const bool both_leg_contact_state_zero =
            (l_leg_contact_state_ == 0) && (r_leg_contact_state_ == 0);

        if(both_leg_contact_state_zero)
        {
            StateBase::msfb_.UpdateCurrentPose(1);
        }
        else
        {
            StateBase::msfb_.UpdateCurrentPose(0);
        }
        return res;
    }
    void SafeController::CheckPostureLimit(){
        if (!posture_limit_enable_) return;
        if (ri_ptr_->hstop_status) return;

        robot_error_state_.posture_error = 0;

        // 获取 IMU roll/pitch 角（rad -> deg）
        Vec3f rpy = ri_ptr_->GetImuRpy();
        constexpr float kRadToDeg = 180.0f / M_PI;
        float roll_deg = rpy(0) * kRadToDeg;
        float pitch_deg = rpy(1) * kRadToDeg;

        const bool exceeded = (std::abs(roll_deg) > roll_threshold_deg_) ||
                              (std::abs(pitch_deg) > pitch_threshold_deg_);

        if (exceeded) {
            posture_exceed_count_++;
            if (posture_exceed_count_ >= posture_exceed_threshold_) {
                posture_limit_exceeded_ = true;
                robot_error_state_.posture_error = 1;
            }
        } else {
            posture_exceed_count_ = 0;
            posture_limit_exceeded_ = false;
        }

        // 姿态异常故障上报
        if (report_body_pose_err_) {
            const bool posture_fault = (robot_error_state_.posture_error == 1);
            const uint32_t body_pose_err_resources = posture_fault ? 1u : 0u;
            std::vector<uint8_t> severity = {0};
            severity[0] = posture_fault ? body_pose_err_fault_severity_ : 0;
            reportFault(body_pose_err_resources,
                               BODY_POSE_ERR,
                               severity,
                               false,
                               "body_pose");
        }
    }
    // ── IsDriverStatusNormal 已迁移至 JointMonitor ──

    bool SafeController::IsMotorTempertureNormal(){
        auto r = motor_temp_.check(ri_ptr_->hstop_status);
        joint_overheat_warn_code  = r.warn_code;
        joint_overheat_error_code = r.error_code;
        joint_overheat_fatal_code = r.fatal_code;
        joint_overheat_notice_code = r.notice_code;
        robot_error_state_.motor_heat_warn = (r.fatal_code != 0) ? 1 : 0;
        return (r.fatal_code == 0);
    }

    bool SafeController::IsDriverTempertureNormal(){
        auto r = driver_temp_.check(ri_ptr_->hstop_status);
        driver_overheat_warn_code  = r.warn_code;
        driver_overheat_error_code = r.error_code;
        driver_overheat_fatal_code = r.fatal_code;
        driver_overheat_notice_code = r.notice_code;
        robot_error_state_.driver_heat_warn = (r.fatal_code != 0) ? 1 : 0;
        return (r.fatal_code == 0);
    }

    void SafeController::SampleTemperatures(){
        motor_temp_.sample(ri_ptr_->GetMotorTemperture());
        driver_temp_.sample(ri_ptr_->GetDriverTemperture());
        if (run_cnt_ % 5000 == 0) {
            motor_temp_.print_raw(ri_ptr_->GetMotorTemperture(), "Motor");
            driver_temp_.print_raw(ri_ptr_->GetDriverTemperture(), "Driver");
        }
    }

    bool SafeController::IsBatteryNormal(){
        std::vector<uint16_t> battery_data = ri_ptr_->GetBatteryData();
        auto battery_info_ = ri_ptr_->GetBatteryInfo();

        // 电量低于10%
        if(battery_data[0] < 10) {
            // std::cout << "Battery level is " << battery_data[0] << std::endl;
            robot_error_state_.battery_low_warn = 1;
            return false;
        }else{
            robot_error_state_.battery_low_warn = 0;
        }
        return true;    
    }
    bool SafeController::IsUserCommandNormal(){
        double cmd_ts_ = uc_ptr_->GetUserCommandSnapshot().time_stamp;
        if(cmd_check_flag_){
            cmd_check_flag_ = false;
            last_cmd_ts_ = cmd_ts_;
            cmd_check_time_ = current_time_;
        }
        robot_error_state_.heatbeat_overtime = 0;
        // 接收命令时间相比上次有变化就会更新
        if(last_cmd_ts_ != cmd_ts_){
            last_cmd_ts_ = cmd_ts_;
            cmd_check_time_ = current_time_;
        }
        // std::cout <<"cmd_ts: "<<cmd_ts_<<std::endl;
        // std::cout <<"delta time : "<< current_time_ - cmd_check_time_ << std::endl;
        // std::cout <<"current: "<< current_time_ << std::endl;
        
        // WiFi断连保护(根据收命令的更新来判断)
        if(current_time_ - cmd_check_time_ > 1000.0) {
            robot_error_state_.heatbeat_overtime = 1;
            // cmd_check_flag_ = true;
            return false;
        }
        return true;
    }


    void SafeController::PrintRobotErrorState(){
        if(robot_error_state_.imu_num_error){
            std::cout << "imu_num_error: " << ri_ptr_->GetImuRpy().transpose() << " | " 
                                            << ri_ptr_->GetImuAcc().transpose() << " | "
                                            << ri_ptr_->GetImuOmega().transpose() << std::endl;
        }
        if(robot_error_state_.imu_update_overtime){
            std::cout << "imu update overtime: " << last_imu_ts_ << std::endl;
        }

        if(robot_error_state_.joint_num_error){
            std::cout << "joint_data_error : \n" << ri_ptr_->GetJointPosition().transpose() << "\n"
                                                 << ri_ptr_->GetJointVelocity().transpose() << "\n"
                                                 << ri_ptr_->GetJointTorque().transpose() << "\n";
        }
        if(robot_error_state_.joint_update_error){
            std::cout << "joint_update_error : " << std::hex << current_time_ << " (ri_check_time moved to JointMonitor)" << std::endl;
        }
        if(robot_error_state_.motor_heat_warn){
            std::cout << "motor_over_heat : " << motor_temp_.max_temp_idx() << " | " << motor_temp_.max_temp() << std::endl;
        }
        if(robot_error_state_.driver_heat_warn){
            std::cout << "driver_over_heat : " << driver_temp_.max_temp_idx() << " | " << driver_temp_.max_temp() << std::endl;
        }

        if(robot_error_state_.driver_error){
            std::cout << "driver_error: \n";
            for(const auto& pair : error_driver_map_){
                std::cout << int(pair.first) << " " << std::hex << int(pair.second) << std::endl;
            }
        }

        if(robot_error_state_.battery_low_warn){
            std::cout << "Battery is in low level" << std::endl;
        }
        if(robot_error_state_.battery_heat_warn){

        }
    }

    SafeController::SafeController(RobotType robot_type, const std::string& path):robot_type_(robot_type), safe_logger_("safeCtl"){
        robot_error_state_.error_code = 0;
        last_error_code_ = 0;
        error_driver_map_.clear();
        fault_client_ = std::make_unique<FaultClient>("motion_master");

        // 读取 [ecan_hardware] with_neck 配置（与 EcanHardwareInterface 保持一致）
        {
            std::string config_toml;
            if (!path.empty()) {
                config_toml = path + "/config.toml";
            } else if (std::filesystem::exists("/var/opt/robot/conf/motion_master/config.toml")) {
                config_toml = "/var/opt/robot/conf/motion_master/config.toml";
            } else {
                config_toml = "conf/config.toml";
            }
            try {
                auto data = toml::parse(config_toml);
                const auto& eh = toml::find(data, "ecan_hardware");
                with_neck_ = toml::find_or<bool>(eh, "with_neck", true);
                safe_logger_.info("[ecan_hardware] with_neck=%s (from %s)",
                                 with_neck_ ? "true" : "false", config_toml.c_str());
            } catch (const std::exception& e) {
                safe_logger_.warn("[ecan_hardware] config read failed: %s, with_neck=true by default", e.what());
                with_neck_ = true;
            }
        }

        // 故障注入 DDS 订阅（调试/测试用，已关闭）
        // fault_inject_sub_ = std::make_unique<ChannelString>(
        //     std::bind(&SafeController::OnFaultInjectCommand, this, std::placeholders::_1),
        //     "/motion_master/fault_inject", 0);
        // safe_logger_.info("Fault injection subscriber created on rt/motion_master/fault_inject");

        LoadFaultConfig();
        LoadPostureLimitConfig();
    }

    void SafeController::SetRobotDataSource(std::shared_ptr<interface::RobotInterface> ri){
        ri_ptr_ = ri;

        // Initialize JointMonitor
        joint_monitor_.init(ri_ptr_->dof_num_, joint_monitor_cfg_);

        // Initialize per-joint LevelMonitor from stored config
        safe_logger_.info("Initializing motor temp monitors: cfg %s",
            motor_temp_cfg_.has_value() ? "loaded" : "NOT loaded (monitors will use defaults)");
        safe_logger_.info("Initializing driver temp monitors: cfg %s",
            driver_temp_cfg_.has_value() ? "loaded" : "NOT loaded (monitors will use defaults)");
        motor_temp_.init(ri_ptr_->dof_num_, motor_temp_cfg_.value_or(LevelMonitorConfig{}));
        driver_temp_.init(ri_ptr_->dof_num_, driver_temp_cfg_.value_or(LevelMonitorConfig{}));
    }
    void SafeController::SetUserCommandDataSource(std::shared_ptr<interface::UserCommandInterface> uc){
        uc_ptr_ = uc;
        usr_cmd_ = uc_ptr_->GetUserCommand();
        // usr_cmd_->safe_control_mode = 0;
    }
    void SafeController::SetControlParamInfaceSource(std::shared_ptr<ControlParameters> cp_ptr){
        cp_ptr_ = cp_ptr;
        joint_pos_lower.resize(cp_ptr_->dof_num_); 
        joint_pos_upper.resize(cp_ptr_->dof_num_);
        if(cp_ptr_->dof_num_ == 31 || cp_ptr_->dof_num_ == 30) {
            joint_pos_lower << cp_ptr_->waist_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->leg_joint_lower_, cp_ptr_->leg_joint_lower_, cp_ptr_->neck_joint_lower_;
            joint_pos_upper << cp_ptr_->waist_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->leg_joint_upper_, cp_ptr_->leg_joint_upper_, cp_ptr_->neck_joint_upper_;
            joint_pos_lower(11) = -cp_ptr_->arm_joint_upper_(1);
            joint_pos_upper(11) = -cp_ptr_->arm_joint_lower_(1);
            joint_pos_lower(12) = -cp_ptr_->arm_joint_upper_(2);
            joint_pos_upper(12) = -cp_ptr_->arm_joint_lower_(2);
            joint_pos_lower(24) = -cp_ptr_->leg_joint_upper_(1);
            joint_pos_upper(24) = -cp_ptr_->leg_joint_lower_(1);
            joint_pos_lower(25) = -cp_ptr_->leg_joint_upper_(2);
            joint_pos_upper(25) = -cp_ptr_->leg_joint_lower_(2);
        }else if(cp_ptr_->dof_num_ == 21) {
            joint_pos_lower << cp_ptr_->waist_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->arm_joint_lower_, cp_ptr_->leg_joint_lower_, cp_ptr_->leg_joint_lower_;
            joint_pos_upper << cp_ptr_->waist_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->arm_joint_upper_, cp_ptr_->leg_joint_upper_, cp_ptr_->leg_joint_upper_;
            joint_pos_lower(6) = -cp_ptr_->arm_joint_upper_(1);
            joint_pos_upper(6) = -cp_ptr_->arm_joint_lower_(1);
            joint_pos_lower(7) = -cp_ptr_->arm_joint_upper_(2);
            joint_pos_upper(7) = -cp_ptr_->arm_joint_lower_(2);
            joint_pos_lower(16) = -cp_ptr_->leg_joint_upper_(1);
            joint_pos_upper(16) = -cp_ptr_->leg_joint_lower_(1);
            joint_pos_lower(17) = -cp_ptr_->leg_joint_upper_(2);
            joint_pos_upper(17) = -cp_ptr_->leg_joint_lower_(2);
        }else{
            joint_pos_lower << cp_ptr_->leg_joint_lower_, cp_ptr_->leg_joint_lower_;
            joint_pos_upper << cp_ptr_->leg_joint_upper_, cp_ptr_->leg_joint_upper_;
            joint_pos_lower(7) = -cp_ptr_->leg_joint_upper_(1);
            joint_pos_upper(7) = -cp_ptr_->leg_joint_lower_(1);
            joint_pos_lower(8) = -cp_ptr_->leg_joint_upper_(2);
            joint_pos_upper(8) = -cp_ptr_->leg_joint_lower_(2);
        }
    }
    void SafeController::Start(){
        start_thread_flag_ = true;
        judge_thread_ = std::thread(std::bind(&SafeController::Run, this));
    }

    void SafeController::Run(){
        int tfd;    //定时器描述符
        int efd;    //epoll描述符
        int fds;
        uint64_t value;
        struct epoll_event ev;  // 用于 epoll_ctl 注册
        struct epoll_event rev; // 用于 epoll_wait 接收
        struct itimerspec time_intv; //用来存储时间

        tfd = timerfd_create(CLOCK_MONOTONIC, 0);   //创建定时器
        if(tfd == -1)  return ;
        
        time_intv.it_value.tv_sec = 1.; 
        time_intv.it_value.tv_nsec = 1000*1000;
        time_intv.it_interval.tv_sec = 0;   
        time_intv.it_interval.tv_nsec = time_intv.it_value.tv_nsec;

        timerfd_settime(tfd, 0, &time_intv, NULL);  //启动定时器

        efd = epoll_create1(0); //创建epoll实例
        if (efd == -1) { close(tfd); return; }

        ev.data.fd = tfd; 
        ev.events = EPOLLIN;    //监听定时器读事件，当定时器超时时，定时器描述符可读。
        epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev); //添加到epoll监听队列中
        while(start_thread_flag_){
            fds = epoll_wait(efd, &rev, 1, -1);    //阻塞监听，直到有事件发生
            if(rev.events & EPOLLIN){   
                int ret = read(rev.data.fd, &value, sizeof(uint64_t));
                if (ret == -1) {
                    continue;
                }
            }
            run_cnt_++;
            uint32_t safe_control_mode = 0;
            current_time_ = GetTimestampMs();
            ri_ts_ = ri_ptr_->GetInterfaceTimeStamp();// / 1000.;
            joint_pos = ri_ptr_->GetJointPosition();
            joint_vel = ri_ptr_->GetJointVelocity();
            joint_tau = ri_ptr_->GetJointTorque();

            // ── 关节故障 + 驱动器状态（委托 JointMonitor）──
            const int leg_start = cp_ptr_->waist_dof_num_ + 2 * cp_ptr_->arm_dof_num_;
            const int leg_end   = static_cast<int>(ri_ptr_->dof_num_) - cp_ptr_->neck_dof_num_;
            {
                auto status_word = ri_ptr_->GetDriverStatusWord();
                bool hstop = ri_ptr_->hstop_status;

                std::vector<uint8_t> upd_sev, nan_sev, pos_sev, vel_sev;
                uint32_t upd_mask = 0, nan_mask = 0, pos_mask = 0, vel_mask = 0, drv_mask = 0;
                uint32_t drv_over_volt=0, drv_under_volt=0, drv_over_curr=0;
                uint32_t drv_motor_temp=0, drv_temp=0, drv_timeout=0;
                uint32_t drv_temp_sensor=0, drv_enc_fail=0, drv_enc_offline=0, drv_discon=0;
                std::map<int, uint16_t> err_map;
                bool not_updated = false;

                if (!hstop) {
                    upd_mask = joint_monitor_.checkDataUpdate(current_time_, ri_ts_,
                        joint_pos, joint_vel, joint_tau, upd_sev, not_updated);
                    nan_mask = joint_monitor_.checkNan(joint_pos, joint_vel, joint_tau, nan_sev);
                    pos_mask = joint_monitor_.checkPosLimit(joint_pos, joint_pos_lower, joint_pos_upper, pos_sev);
                    vel_mask = joint_monitor_.checkVelLimit(joint_vel, cp_ptr_->joint_vel_limit_, vel_sev);
                    drv_mask = joint_monitor_.checkDriverStatus(status_word,
                        drv_over_volt, drv_under_volt, drv_over_curr,
                        drv_motor_temp, drv_temp, drv_timeout,
                        drv_temp_sensor, drv_enc_fail, drv_enc_offline, drv_discon, err_map);
                }

                // 写入 atomic（供 GetFaultSnapshot 跨线程读取）
                joint_update_state_.store(upd_mask, std::memory_order_release);
                joint_nan_code.store(nan_mask, std::memory_order_release);
                joint_pos_error_code.store(pos_mask, std::memory_order_release);
                joint_vel_error_code = vel_mask;
                driver_status_error_code.store(drv_mask, std::memory_order_release);
                joint_data_not_updated_ = not_updated;
                error_driver_map_ = std::move(err_map);
                motor_over_temp_mask_from_driverstatus_ = drv_motor_temp;
                driver_over_temp_mask_from_driverstatus_ = drv_temp;

                // robot_error_state_
                robot_error_state_.joint_update_error = upd_mask ? 1 : 0;
                robot_error_state_.joint_num_error = (nan_mask || pos_mask || vel_mask) ? 1 : 0;
                robot_error_state_.driver_error = drv_mask ? 1 : 0;
                if (drv_mask == 0) error_driver_map_.clear();

                // safe_control_mode
                if (upd_mask) safe_control_mode |= (1 << kSafeJointUpdate);
                if (drv_mask) safe_control_mode |= (1 << kSafeDriverStatus);
                if (pos_mask) safe_control_mode |= (1 << kSafeJointPosLimit);
                if (nan_mask) safe_control_mode |= (1 << kSafeJointNan);

                // reportFault（严重等级由 LoadFaultConfig 从故障配置动态读取，支持 custom 覆盖）
                // 腰/腿关节故障升级：ERROR → CRITICAL（pos/vel/NAN/update joints）
                {
                    auto upgradeWaistLegInVector = [&](uint32_t mask, std::vector<uint8_t>& sev) {
                        size_t idx = 0;
                        for (int i = 0; i < static_cast<int>(ri_ptr_->dof_num_); ++i) {
                            if (mask & (1u << i)) {
                                bool wl = (i < cp_ptr_->waist_dof_num_) || (i >= leg_start && i < leg_end);
                                if (wl && sev[idx] == Severity::ERROR)
                                    sev[idx] = Severity::CRITICAL;
                                ++idx;
                            }
                        }
                    };
                    upgradeWaistLegInVector(upd_mask, upd_sev);
                    upgradeWaistLegInVector(nan_mask, nan_sev);
                    upgradeWaistLegInVector(pos_mask, pos_sev);
                    upgradeWaistLegInVector(vel_mask, vel_sev);
                }
                if (report_joint_data_update_error_)  reportFault(upd_mask, JNT_DATA_UPD_ERR, upd_sev, true);
                if (joint_nan_cfg_.enabled)           reportFault(nan_mask, JNT_DATA_NAN, nan_sev, true);
                if (pos_over_limit_cfg_.enabled)      reportFault(pos_mask, JNT_POS_OVER_LIM, pos_sev, true);
                if (vel_over_limit_cfg_.enabled)      reportFault(vel_mask, JNT_VEL_OVER_LIM, vel_sev, true);
                if (report_joint_driver_over_volt_)   reportFault(drv_over_volt,  JNT_DRV_OVER_VOLT,  {joint_drv_over_volt_severity_}, true);
                if (report_joint_driver_under_volt_)  reportFault(drv_under_volt, JNT_DRV_UNDER_VOLT, {joint_drv_under_volt_severity_}, true);
                if (report_joint_driver_over_curr_)   reportFault(drv_over_curr,  JNT_DRV_OVER_CURR,  {joint_drv_over_curr_severity_}, true);

                // 腰/腿关节故障升级：ERROR → CRITICAL（joint timeout, encoder no value, driver status error）
                {
                    auto sevWithWaistLegUpgrade = [&](uint32_t mask, uint8_t base) -> std::vector<uint8_t> {
                        std::vector<uint8_t> out;
                        for (int i = 0; i < static_cast<int>(ri_ptr_->dof_num_); ++i) {
                            if (mask & (1u << i)) {
                                bool wl = (i < cp_ptr_->waist_dof_num_) || (i >= leg_start && i < leg_end);
                                out.push_back((wl && base == Severity::ERROR) ? Severity::CRITICAL : base);
                            }
                        }
                        return out;
                    };
                    if (report_joint_driver_timeout_)
                        reportFault(drv_timeout, JNT_DRV_TIMEOUT,
                                    sevWithWaistLegUpgrade(drv_timeout, joint_drv_timeout_severity_), true);
                    if (report_joint_enc_no_val_)
                        reportFault(drv_enc_offline, JNT_ENC_NO_VAL,
                                    sevWithWaistLegUpgrade(drv_enc_offline, joint_enc_no_val_severity_), true);
                    if (report_joint_driver_status_error_)
                        reportFault(drv_discon, JNT_DRV_STATUS_ERR,
                                    sevWithWaistLegUpgrade(drv_discon, joint_drv_status_err_severity_), true);
                }

                if (report_temp_sensor_discon_)       reportFault(drv_temp_sensor,TEMP_SENSOR_DISCON,  {temp_sensor_discon_severity_}, true);
                if (report_enc_calib_fail_)           reportFault(drv_enc_fail,   ENC_CALIB_FAIL,     {enc_calib_fail_severity_}, true);
            }

            // BODY_POSE_ERR: use GroundContactDetection in selected gaits.
            if (!IsGroundContactNormal()) {
                safe_control_mode |= (1 << kSafePostureNormal);
            }
            // 姿态超限检测（roll/pitch角超阈值）
            CheckPostureLimit();
            if (posture_limit_exceeded_) {
                safe_control_mode |= (1 << kSafePostureNormal);
            }

            if(run_cnt_%1000==0){
                if(!IsMotorTempertureNormal())
                {
                    safe_control_mode |= (1 << kSafeMotorTemp);
                }
                if(!IsDriverTempertureNormal())
                {
                    safe_control_mode |= (1 << kSafeDriverTemp);
                }
                // IsBatteryNormal();
            } else {
                // Sample temperatures at 1ms rate, average computed every 1s in IsXxxTempertureNormal()
                SampleTemperatures();
            }

            // Merge and publish MOTOR_OVER_TEMP / JNT_DRV_OVER_TEMP only once per loop
            if (report_motor_over_temp_ || report_joint_driver_over_temp_) {
                // 腰部和腿部关节温度故障升级：NOTICE→WARN, WARN→ERROR, ERROR→CRITICAL
                auto upgradeWaistLegSev = [&](int idx, uint8_t sev) -> uint8_t {
                    bool is_waist_or_leg = (idx < cp_ptr_->waist_dof_num_) || (idx >= leg_start && idx < leg_end);
                    if (!is_waist_or_leg) return sev;
                    if (sev == Severity::NOTICE) return Severity::WARN;
                    if (sev == Severity::WARN)   return Severity::ERROR;
                    if (sev == Severity::ERROR)  return Severity::CRITICAL;
                    return sev;
                };

                // motor_over_temp
                if (report_motor_over_temp_) {
                    uint32_t combined_resources =
                        joint_overheat_notice_code | joint_overheat_warn_code | joint_overheat_error_code | joint_overheat_fatal_code | motor_over_temp_mask_from_driverstatus_;
                    std::vector<uint8_t> combined_sev;
                    combined_sev.reserve(ri_ptr_->dof_num_);
                    for (int i = 0; i < static_cast<int>(ri_ptr_->dof_num_); ++i) {
                        uint8_t sev = 0;
                        if (motor_over_temp_mask_from_driverstatus_ & (1u << i)) {
                            sev = Severity::FATAL;
                        } else if (joint_overheat_fatal_code & (1u << i)) {
                            sev = Severity::FATAL;
                        } else if (joint_overheat_error_code & (1u << i)) {
                            sev = Severity::ERROR;
                        } else if (joint_overheat_warn_code & (1u << i)) {
                            sev = Severity::WARN;
                        } else if (joint_overheat_notice_code & (1u << i)) {
                            sev = Severity::NOTICE;
                        }
                        if (sev) combined_sev.push_back(upgradeWaistLegSev(i, sev));
                    }
                    reportFault(combined_resources,
                                       MOTOR_OVER_TEMP,
                                       combined_sev,
                                       true);
                }

                // driver over temp
                if (report_joint_driver_over_temp_) {
                    uint32_t combined_resources =
                        driver_overheat_notice_code | driver_overheat_warn_code | driver_overheat_error_code | driver_overheat_fatal_code | driver_over_temp_mask_from_driverstatus_;
                    std::vector<uint8_t> combined_sev;
                    combined_sev.reserve(ri_ptr_->dof_num_);
                    for (int i = 0; i < static_cast<int>(ri_ptr_->dof_num_); ++i) {
                        uint8_t sev = 0;
                        if (driver_over_temp_mask_from_driverstatus_ & (1u << i)) {
                            sev = Severity::FATAL;
                        } else if (driver_overheat_fatal_code & (1u << i)) {
                            sev = Severity::FATAL;
                        } else if (driver_overheat_error_code & (1u << i)) {
                            sev = Severity::ERROR;
                        } else if (driver_overheat_warn_code & (1u << i)) {
                            sev = Severity::WARN;
                        } else if (driver_overheat_notice_code & (1u << i)) {
                            sev = Severity::NOTICE;
                        }
                        if (sev) combined_sev.push_back(upgradeWaistLegSev(i, sev));
                    }
                    reportFault(combined_resources,
                                       JNT_DRV_OVER_TEMP,
                                       combined_sev,
                                       true);
                }
            }

            // IsRobotWarnMode();
            if(!IsImuDataNormal()){
                safe_control_mode |= (1 << kSafeImuData);
            }

            // 故障注入掩码（调试/测试用）：将注入掩码映射为 safe_control_mode 位
            // 使注入故障能触发状态机分级降级流程（同真实硬件故障）
            if (injected_driver_status_code_.load(std::memory_order_acquire)) {
                safe_control_mode |= (1 << kSafeDriverStatus);
            }
            if (injected_update_state_.load(std::memory_order_acquire)) {
                safe_control_mode |= (1 << kSafeJointUpdate);
            }
            if (injected_nan_code_.load(std::memory_order_acquire)) {
                safe_control_mode |= (1 << kSafeJointNan);
            }
            if (injected_pos_error_code_.load(std::memory_order_acquire)) {
                safe_control_mode |= (1 << kSafeJointPosLimit);
            }
            // 将 robot_error_state_ 快照到原子变量，供 GetFaultSnapshot/ClassifyFault 跨线程安全读取
            snapshot_error_code_.store(robot_error_state_.error_code, std::memory_order_release);
            snapshot_posture_error_.store(robot_error_state_.posture_error != 0, std::memory_order_release);
            // if(!IsUserCommandNormal()){
            //     usr_cmd_->safe_control_mode = 2;
            // } else {
            //     usr_cmd_->safe_control_mode = 0;
            // }
            // Tangxia/Zhanli intentionally exceed the standing posture limit.
            // Suppress only the posture soft-stop bit in these two states; all
            // IMU data, joint, driver, temperature and communication checks
            // continue to reach the state machine unchanged.
            if (RecoveryPostureExceptionActive()) {
                safe_control_mode &= ~(1u << kSafePostureNormal);
            }
            // 暂停模式：使能阶段等场景下强制清零，避免误触发软急停
            // 仅上报模式：不向下游传递故障位，避免状态机触发软急停；
            // 故障上报（reportFault）和本地诊断日志照常输出。
            if (paused_.load(std::memory_order_acquire) || report_only_) {
                uc_ptr_->SetSafeControlMode(0);
            } else {
                uc_ptr_->SetSafeControlMode(safe_control_mode);
            }

            if(last_error_code_ != safe_control_mode){
                if(safe_control_mode != 0){
                    if (report_only_) {
                        safe_logger_.warn("[REPORT_ONLY] error_code: %u (soft e-stop suppressed)", safe_control_mode);
                    } else {
                        safe_logger_.error("error_code: %u", safe_control_mode);
                    }
                    PrintRobotErrorState();
                }
                last_error_code_ = safe_control_mode;
            }
        }
        close(tfd);
        close(efd);
    }

SafeController::FaultSnapshot SafeController::GetFaultSnapshot() const {
    FaultSnapshot snap;
    snap.error_code         = snapshot_error_code_.load(std::memory_order_acquire);
    snap.pos_error_mask     = joint_pos_error_code.load(std::memory_order_acquire);
    snap.nan_mask           = joint_nan_code.load(std::memory_order_acquire);
    snap.update_mask        = joint_update_state_.load(std::memory_order_acquire);
    snap.driver_status_mask = driver_status_error_code.load(std::memory_order_acquire);
    snap.imu_update_timeout = (imu_update_timeout_flag_.load(std::memory_order_acquire) != 0);
    snap.imu_data_nan       = (imu_data_nan_flag_.load(std::memory_order_acquire) != 0);
    snap.posture_error      = snapshot_posture_error_.load(std::memory_order_acquire);
    return snap;
}

uint32_t SafeController::GetFaultMask() {
    return (joint_pos_error_code.load(std::memory_order_acquire) | injected_pos_error_code_.load(std::memory_order_acquire))
         | (joint_nan_code.load(std::memory_order_acquire) | injected_nan_code_.load(std::memory_order_acquire))
         | (joint_update_state_.load(std::memory_order_acquire) | injected_update_state_.load(std::memory_order_acquire))
         | (driver_status_error_code.load(std::memory_order_acquire) | injected_driver_status_code_.load(std::memory_order_acquire));
}

BodyFaultGroup SafeController::ClassifyFault(uint32_t fault_mask) {
    // 关节索引范围计算
    int offset = 0;
    offset += cp_ptr_->waist_dof_num_;  // waist: [0, waist_dof_num_)
    int larm_start = offset;
    offset += cp_ptr_->arm_dof_num_;
    offset += cp_ptr_->arm_dof_num_;
    int rarm_end = offset - 1;
    int lleg_start = offset;
    offset += cp_ptr_->leg_dof_num_;
    offset += cp_ptr_->leg_dof_num_;
    int rleg_end = offset - 1;
    int neck_start = offset;
    int neck_end = offset + cp_ptr_->neck_dof_num_ - 1;

    // 检查腿部
    for (int i = lleg_start; i <= rleg_end; i++) {
        if (fault_mask & (1u << i)) return BodyFaultGroup::LEG;
    }

    // 检查腰X/Y（仅3DOF腰部有X/Y）
    if (cp_ptr_->waist_dof_num_ >= 3) {
        if (fault_mask & ((1u << 0) | (1u << 1))) return BodyFaultGroup::WAIST_XY;
    }

    // 检查底盘IMU
    if (snapshot_error_code_.load(std::memory_order_acquire) & 0x03) return BodyFaultGroup::IMU_BODY;

    // 检查头部
    if (cp_ptr_->neck_dof_num_ > 0) {
        for (int i = neck_start; i <= neck_end; i++) {
            if (fault_mask & (1u << i)) return BodyFaultGroup::HEAD;
        }
    }

    // 检查臂部
    for (int i = larm_start; i <= rarm_end; i++) {
        if (fault_mask & (1u << i)) return BodyFaultGroup::UPPER_LIMB;
    }

    // 检查腰Z
    if (cp_ptr_->waist_dof_num_ >= 3) {
        if (fault_mask & (1u << 2)) return BodyFaultGroup::WAIST_Z;
    } else if (cp_ptr_->waist_dof_num_ == 1) {
        if (fault_mask & (1u << 0)) return BodyFaultGroup::WAIST_Z;
    }

    return BodyFaultGroup::NONE;
}

// ===== 故障注入接口实现 =====

/*  // OnFaultInjectCommand — 调试用，声明已注释
void SafeController::OnFaultInjectCommand(const std_msgs::msg::String* msg) {
    if (!msg) return;
    const std::string& s = msg->data();

    // 解析: "SET <type> <hex>" 或 "CLEAR <type>" 或 "CLEAR ALL"
    std::istringstream iss(s);
    std::string action, type, val;
    iss >> action >> type >> val;

    uint32_t mask = 0;
    if (!val.empty()) {
        mask = std::stoul(val, nullptr, 16);
    }

    auto apply = [&](std::atomic<uint32_t>& target, const char* name) {
        if (action == "SET") {
            target.store(mask, std::memory_order_release);
            safe_logger_.info("[FaultInject] SET %s = 0x%X", name, mask);
        } else if (action == "CLEAR") {
            target.store(0, std::memory_order_release);
            safe_logger_.info("[FaultInject] CLEAR %s", name);
        }
    };

    if (type == "pos_over_limit") {
        apply(injected_pos_error_code_, "pos_over_limit");
    } else if (type == "nan") {
        apply(injected_nan_code_, "nan");
    } else if (type == "update_err") {
        apply(injected_update_state_, "update_err");
    } else if (type == "driver_status") {
        apply(injected_driver_status_code_, "driver_status");
    } else if (type == "ALL" && action == "CLEAR") {
        ClearInjectedFaultMask();
        safe_logger_.info("[FaultInject] CLEAR ALL");
    } else {
        safe_logger_.warn("[FaultInject] Unknown command: %s", s.c_str());
    }
}
*/

void SafeController::SetInjectedFaultMask(uint32_t pos, uint32_t nan, uint32_t upd, uint32_t drv) {
    injected_pos_error_code_.store(pos, std::memory_order_release);
    injected_nan_code_.store(nan, std::memory_order_release);
    injected_update_state_.store(upd, std::memory_order_release);
    injected_driver_status_code_.store(drv, std::memory_order_release);
}

void SafeController::ClearInjectedFaultMask() {
    injected_pos_error_code_.store(0, std::memory_order_release);
    injected_nan_code_.store(0, std::memory_order_release);
    injected_update_state_.store(0, std::memory_order_release);
    injected_driver_status_code_.store(0, std::memory_order_release);
}

uint32_t SafeController::GetInjectedFaultMask() const {
    return injected_pos_error_code_.load(std::memory_order_acquire)
         | injected_nan_code_.load(std::memory_order_acquire)
         | injected_update_state_.load(std::memory_order_acquire)
         | injected_driver_status_code_.load(std::memory_order_acquire);
}

/* 构建 (can_ch, can_id) → joint_index 映射表（懒加载，首次调用时执行） */
void SafeController::buildMotorCommJointMap() {
    try {
        RobotConfig hwi_config;
        std::string model = hwi_config.getModel();
        auto joint_map = hwi_config.getMapForModel(model);
        motor_comm_joint_map_.clear();
        for (const auto& [idx, xyz] : joint_map) {
            uint16_t key = ((uint16_t)xyz.can_ch << 4) | (uint16_t)(xyz.can_id & 0x0F);
            motor_comm_joint_map_[key] = (int)idx;
        }
        safe_logger_.info("MotorComm joint map built: model=%s, %zu entries",
                          model.c_str(), motor_comm_joint_map_.size());
    } catch (const std::exception& e) {
        safe_logger_.error("buildMotorCommJointMap failed: %s", e.what());
    }
    motor_comm_map_init_ = true;
}

/* 电机通信状态故障检测与上报（由 handlMotorCommStatus 回调调用，ecan 接收线程） */
void SafeController::CheckMotorCommStatus(const ecan::MotorCommStatusResponse& status) {
    if (!motor_comm_map_init_) {
        buildMotorCommJointMap();
    }

    /* 1. 增量合并到全量缓存（MCU 只上报有变化的电机） */
    uint8_t n = status.count;
    if (n > 128) n = 128;
    for (uint8_t i = 0; i < n; i++) {
        const auto& m = status.motors[i];
        uint16_t key = ((uint16_t)m.can_ch << 4) | (uint16_t)(m.can_id & 0x0F);
        auto& entry = motor_comm_cache_[key];
        entry.enable_status = m.enable_status;
        entry.err_recent = m.err_frame_recent;
        entry.loss_recent = m.pkt_loss_recent;
        entry.bus_off = m.bus_off;
    }

    /* 2. 基于全量缓存计算 4 类故障的关节位图 */
    uint32_t disabled_mask = 0, err_mask = 0, loss_mask = 0, busoff_mask = 0;
    for (const auto& [key, entry] : motor_comm_cache_) {
        auto it = motor_comm_joint_map_.find(key);
        if (it == motor_comm_joint_map_.end()) continue;   // 未映射的电机跳过
        int idx = it->second;
        if (idx < 0 || idx >= 32) continue;

        // 未配置 neck 时，跳过 neck 通道（ch=1）的故障
        if (!with_neck_) {
            uint8_t can_ch = (key >> 4) & 0x0F;
            if (can_ch == 1) continue;
        }

        if (entry.enable_status == 0) disabled_mask |= (1u << idx);
        if (entry.err_recent >= motor_comm_err_threshold_)   err_mask      |= (1u << idx);
        if (entry.loss_recent >= motor_comm_loss_threshold_)  loss_mask     |= (1u << idx);
        if (entry.bus_off != 0)       busoff_mask   |= (1u << idx);
    }

    /* 3. 根据 config.toml [ecan_hardware] with_neck 过滤 neck 关节故障上报 */
    if (!with_neck_ && cp_ptr_->neck_dof_num_ > 0) {
        uint32_t neck_mask = ((1u << cp_ptr_->neck_dof_num_) - 1)
                           << (cp_ptr_->dof_num_ - cp_ptr_->neck_dof_num_);
        disabled_mask &= ~neck_mask;
        err_mask      &= ~neck_mask;
        loss_mask     &= ~neck_mask;
        busoff_mask   &= ~neck_mask;
    }

    /* 4. 上报（reportFault 内部去重：位图不变不发，归零自动发 STOP）
     * 注意：severity 向量的长度必须等于 set bits 数，DEM 服务端在 grouped=true
     * 时会校验 severities.size() == resources.size()，不匹配则丢弃事件。 */
    {
        uint32_t n_disabled = __builtin_popcount(disabled_mask);
        uint32_t n_err      = __builtin_popcount(err_mask);
        uint32_t n_loss     = __builtin_popcount(loss_mask);
        uint32_t n_busoff   = __builtin_popcount(busoff_mask);
        if (report_motor_comm_disabled_) reportFault(disabled_mask, JNT_DRV_DISABLED,
                    std::vector<uint8_t>(n_disabled, motor_comm_disabled_severity_), true);
        if (report_motor_comm_err_)      reportFault(err_mask, JNT_ERR_FRAME_EXCESS,
                    std::vector<uint8_t>(n_err, motor_comm_err_severity_), true);
        if (report_motor_comm_loss_)     reportFault(loss_mask, JNT_LOST_FRAME_EXCESS,
                    std::vector<uint8_t>(n_loss, motor_comm_loss_severity_), true);
        if (report_motor_comm_busoff_)   reportFault(busoff_mask, JNT_BUS_OFF,
                    std::vector<uint8_t>(n_busoff, motor_comm_busoff_severity_), true);
    }
}

/* 主板电压（输入+输出）越限检测与上报（由 handlVoltageStatus 回调调用，1Hz）
 * 输入电压：bat_voltage → MB_INPUT_UNDER_VOLT / MB_INPUT_OVER_VOLT（单一资源）
 * 输出电压：up_voltage/down_voltage → MB_OUTPUT_UNDER_VOLT / MB_OUTPUT_OVER_VOLT（分组 bitmap） */
void SafeController::CheckMainboardVoltage(float bat_voltage, float up_voltage, float down_voltage) {

    /* ---- 输入电压（电池/主板输入）---- */
    // 欠压：< voltage_under_threshold_ 且 > 0.1V（排除掉电/未连接）
    uint32_t in_under_mask = (bat_voltage < voltage_under_threshold_ && bat_voltage > 0.1f) ? 1u : 0u;
    uint32_t in_over_mask  = (bat_voltage > voltage_over_threshold_) ? 1u : 0u;

    if (report_mainboard_input_under_volt_)
        reportFault(in_under_mask, MB_INPUT_UNDER_VOLT,
                    {voltage_input_under_severity_}, false, "103");
    if (report_mainboard_input_over_volt_)
        reportFault(in_over_mask, MB_INPUT_OVER_VOLT,
                    {voltage_input_over_severity_}, false, "103");

    /* ---- 输出电压（上肢/下肢）---- */
    uint32_t out_under_mask = 0, out_over_mask = 0;
    if (up_voltage < voltage_under_threshold_)   out_under_mask |= 0x01u;  // bit0 = 上肢
    if (down_voltage < voltage_under_threshold_) out_under_mask |= 0x02u;  // bit1 = 下肢
    if (up_voltage > voltage_over_threshold_)    out_over_mask  |= 0x01u;
    if (down_voltage > voltage_over_threshold_)  out_over_mask  |= 0x02u;

    {
        uint32_t n_out_under = __builtin_popcount(out_under_mask);
        uint32_t n_out_over  = __builtin_popcount(out_over_mask);
        if (report_mainboard_output_under_volt_)
            reportFault(out_under_mask, MB_OUTPUT_UNDER_VOLT,
                        std::vector<uint8_t>(n_out_under, voltage_output_under_severity_), true);
        if (report_mainboard_output_over_volt_)
            reportFault(out_over_mask, MB_OUTPUT_OVER_VOLT,
                        std::vector<uint8_t>(n_out_over, voltage_output_over_severity_), true);
    }
}


