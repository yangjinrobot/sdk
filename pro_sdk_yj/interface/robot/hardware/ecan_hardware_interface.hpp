/**
 * @file ecan_hardware_interface.hpp
 * @brief hardware interface for ecan
 * @author deeprobotics
 * @version 1.0
 * @date 2024-08-19
 * 
 * @copyright Copyright (c) 2024  DeepRobotics
 * 
 */

#ifndef ECAN_HARDWARE_INTERFACE_HPP_
#define ECAN_HARDWARE_INTERFACE_HPP_

#include "common_types.h"
#include "dds_types.h"
#include "robot_interface.h"
#include "drdds/core/drdds_core.h"
#include "robot_hwi.hpp"
#include "motor.h"
#include "types/data_structures.h"
#include "wt_dog.h"
#include "snapshot.h"
#include "data_recorder.h"
#include "voltage_recorder.h"
#include "joint_protection.hpp"
#include "runtime_params.hpp"
#include "safe_controller/safe_controller.hpp"
#include "toml/toml.hpp"
#include "robot_model.hpp"

#include <Eigen/Geometry>
#include <filesystem>
#include <mutex>
#include <cmath>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <unistd.h>

struct JointConfig{
    float dir;
    float offset;
};

class EcanHardwareInterface : public RobotInterface{
protected:
    // Lock order when both are required: command -> feedback. Feedback callback
    // locks never cover command dispatch, DDS publication, or recovery joins.
    mutable std::recursive_mutex feedback_mutex_;
    mutable std::recursive_mutex command_mutex_;
    double ri_ts_, imu_leg_ts_, imu_body_ts_;
    Vec3f omega_body_, rpy_body_, acc_body_;
    Vec3f omega_leg_, rpy_leg_, acc_leg_;
    VecXf joint_pos_, joint_vel_, joint_tau_;
    VecXf joint_pos_raw_;
    VecXf motor_temperture_, driver_temperture_;
    float *pos_offset_;
    float *joint_dir_;
    bool *data_updated_;
    JointConfig *joint_config_;
    std::vector<uint16_t> driver_status_;
    std::vector<uint16_t> joint_data_id_;
    BatteryInfo_dds battery_info_[2]{};
    std::vector<uint16_t> battery_data_;
    DataRecorder data_recorder_;
    VoltageRecorder voltage_recorder_;
    VoltageStatus latest_voltage_{};
    std::mutex voltage_mutex_; 
    MotorCommStatusResponse latest_motor_comm_{};
    std::mutex motor_comm_mutex_;
    // 电机使能/标零/bus_off状态变化追踪：按 (can_ch<<4 | can_id) 索引，
    // 支持增量上报（count 可能小于总电机数）
    struct MotorCommPrevState {
        bool valid = true;
        uint8_t enable_status = 0;
        uint8_t set_home_status = 0;
        uint8_t bus_off = 0;
    };
    MotorCommPrevState motor_comm_prev_[256];  // key = (can_ch << 4) | can_id
    // 异常检测状态机：同一异常只记录一次，恢复时记录一次
    enum VoltageFault { kVfNone = 0, kVfZero, kVfUnder, kVfOver };
    int voltage_fault_state_ = kVfNone;
    // 正常运行期间的电压历史统计（故障首次触发时上报，随后清零重启）
    float voltage_hist_max_ = -1e9f;
    float voltage_hist_min_ = 1e9f;
    float voltage_hist_sum_ = 0.0f;
    uint32_t voltage_hist_count_ = 0;
    // 上肢/下肢电压状态机（kVfNone/kVfUnder/kVfOver），仅状态变化时打日志
    int up_volt_fault_state_ = kVfNone;
    int down_volt_fault_state_ = kVfNone;
    double time_stamp_;

    int x_points_num=35, y_points_num=21;
    float body_height=0*0.86f;
    VecXf sim_hmap_sequence;
    VecXf hmap_buffers_[2];
    std::atomic<int> hmap_write_index_{0};
    bool hmap_is_updated = false;

    std::unique_ptr<ChannelImage> hmap_sub_;
    std::unique_ptr<ChannelStdMsgInt32> hes_status_sub_;

    Watchdog *ecan_wd_ = nullptr;
    drdds::msg::Joints joints_data;
    drdds::msg::JointsCmd joints_cmd;
    drdds::msg::ImuData Leg_Imu_data;
    drdds::msg::ImuData arm_Imu_data;
    std::thread data_pub_thread_;
    std::unique_ptr<ChannelImu> ch_imu_head_data_;
    std::unique_ptr<ChannelImu> ch_imu_base_data_;
    std::unique_ptr<ChannelImu> ch_imu_data_;
    std::unique_ptr<ChannelImuData> ch_imu_leg_data_;
    std::unique_ptr<ChannelBatteryData> h_battery_data_;
    std::unique_ptr<ChannelJoints> ch_joints_data;
    std::unique_ptr<ChannelJoints> ch_joints_data_10Hz;
    std::atomic<bool> start_flag_{false};

    // 硬急停恢复检测相关
    std::atomic<bool>             hstop_recover_pending_{false};
    std::atomic<bool>             stop_recover_{false};
    int                           hstop_snapshot_run_cnt_ = 0;  // 记录pending时的run_cnt_快照
    static constexpr int          kHstopRecoverDataFrames = 5000;  // 等待10帧电机数据后触发回调
    std::thread                   hstop_recover_thread_;

    std::shared_ptr<Motor>  ecan_motor;

    DrLogger logger_;
    JointProtectionChecker joint_protection_;
    std::weak_ptr<SafeController> safe_controller_;  // 用于 ResetJointError 中暂停/恢复安全控制

    // 是否携带 neck（头部）关节硬件；false 时对 neck 关节下发假数据并跳过数据不更新校验
    bool with_neck_ = true;

    // 加载 [ecan_hardware] 配置；解析失败时保持默认值（with_neck = true）
    void LoadEcanHardwareConfig() {
        std::string config_path;
        for (const char* candidate : {
                 "/var/opt/robot/conf/motion_master/config.toml",
                 "conf/config.toml"}) {
            if (std::filesystem::exists(candidate)) {
                config_path = candidate;
                break;
            }
        }
        if (config_path.empty()) {
            logger_.warn("[ecan_hardware] config not found, with_neck=true by default");
        } else {
            try {
                auto cfg = toml::parse(config_path);
                const auto& eh = toml::find(cfg, "ecan_hardware");
                with_neck_ = toml::find_or<bool>(eh, "with_neck", true);
                logger_.info("[ecan_hardware] with_neck=%s (from %s)",
                             with_neck_ ? "true" : "false", config_path.c_str());
            } catch (const std::exception& e) {
                logger_.warn("[ecan_hardware] parse failed: %s, with_neck=true by default", e.what());
                with_neck_ = true;
            }
        }
        // 上报实际生效值至运行期参数快照
        RuntimeParams::Instance().SetBool("ecan_hardware", "with_neck", with_neck_);
    }

    void sendDampingCommand() {
        std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
        JointRawData data_send = {0};
        const float kd = joint_protection_.isEnabled()
                         ? joint_protection_.getDampingKd()
                         : 10.0f;
        for (int i = 0; i < kJointCount; i++) {
            data_send[i].joint_data.tor = 0;
            data_send[i].joint_data.pos = 0;
            data_send[i].joint_data.vel = 0;
            data_send[i].joint_data.kp  = 0;
            data_send[i].joint_data.kd  = kd;
            data_send[i].control_word = kIndexMotorControl;
        }
        ecan_motor->SetJointsCMD(data_send);
    }

    void handlEcanStatus(const ECAN_STATUS& ecan_status) 
    {
        hes_status_sub_->data_->value() = ecan_status.hard_stop;
        hes_status_sub_->Write();
        // 硬急停恢复边沿检测：hstop_status(上一次) = 1 -> ecan_status.hard_stop(本次) = 0
        if (hstop_status && !ecan_status.hard_stop) {
            hstop_snapshot_run_cnt_ = run_cnt_;  // 快照当前电机数据帧计数
            hstop_recover_pending_.store(true, std::memory_order_release);
            logger_.info("Hard stop released, waiting for %d motor data frames before recovery callback...",
                         kHstopRecoverDataFrames);
        }
        hstop_status = ecan_status.hard_stop;
        return;
    }

    /* 电压状态回调：更新缓存 + 异常检测。
     * 异常检测采用状态机：同一异常只记录一次（含历史统计），
     * 恢复正常时记录一次恢复日志，然后重新开始统计。
     * 注意：此回调在 ecan_master 的 RunRoutine 接收线程中同步执行，
     * 不可做重 I/O，CSV 写盘由 voltage_recorder_ 独立线程负责。 */
    void handlVoltageStatus(const VoltageStatus& voltage_status)
    {
        // 更新最新电压缓存（供 VoltageRecorder 1Hz 读取）
        {
            std::lock_guard<std::mutex> lock(voltage_mutex_);
            latest_voltage_ = voltage_status;
        }

        // 判断当前故障状态
        int cur_fault;
        if (voltage_status.bat_voltage < 0.1f) {
            cur_fault = kVfZero;
        } else if (voltage_status.bat_voltage < 54.0f) {
            cur_fault = kVfUnder;
        } else if (voltage_status.bat_voltage > 90.0f) {
            cur_fault = kVfOver;
        } else {
            cur_fault = kVfNone;
        }

        if (cur_fault == kVfNone) {
            // 正常状态：更新历史统计
            float v = voltage_status.bat_voltage;
            if (v > voltage_hist_max_) voltage_hist_max_ = v;
            if (v < voltage_hist_min_) voltage_hist_min_ = v;
            voltage_hist_sum_ += v;
            voltage_hist_count_++;

            // 从异常恢复到正常：记录一次恢复
            if (voltage_fault_state_ != kVfNone) {
                const char* fault_name[] = {"", "ZERO", "UNDERVOLTAGE", "OVERVOLTAGE"};
                logger_.info("Voltage RECOVERED from %s, back to normal: bat=%.2fV",
                             fault_name[voltage_fault_state_], v);
                // 清零统计，重新开始
                voltage_hist_max_ = v;
                voltage_hist_min_ = v;
                voltage_hist_sum_ = v;
                voltage_hist_count_ = 1;
                voltage_fault_state_ = kVfNone;
            }
        } else {
            // 异常状态：仅当状态变化时记录一次
            if (voltage_fault_state_ != cur_fault) {
                const char* fault_name[] = {"", "ZERO", "UNDERVOLTAGE", "OVERVOLTAGE"};
                // 计算历史统计（从上次恢复到本次故障）
                float avg = (voltage_hist_count_ > 0)
                            ? voltage_hist_sum_ / voltage_hist_count_ : 0.0f;
                if (cur_fault == kVfZero) {
                    logger_.error("Battery voltage ZERO detected: bat=%.2fV | "
                                  "history: max=%.2fV min=%.2fV avg=%.2fV (n=%u)",
                                  voltage_status.bat_voltage,
                                  voltage_hist_max_, voltage_hist_min_, avg,
                                  voltage_hist_count_);
                } else if (cur_fault == kVfUnder) {
                    logger_.warn("Battery UNDERVOLTAGE: bat=%.2fV < 54V | "
                                 "history: max=%.2fV min=%.2fV avg=%.2fV (n=%u)",
                                 voltage_status.bat_voltage,
                                 voltage_hist_max_, voltage_hist_min_, avg,
                                 voltage_hist_count_);
                } else {
                    logger_.error("Battery OVERVOLTAGE: bat=%.2fV > 90V | "
                                  "history: max=%.2fV min=%.2fV avg=%.2fV (n=%u)",
                                  voltage_status.bat_voltage,
                                  voltage_hist_max_, voltage_hist_min_, avg,
                                  voltage_hist_count_);
                }
                voltage_fault_state_ = cur_fault;
            }
            // 同一异常持续期间不重复记录
        }

        /* === 上肢/下肢电压检测：状态变化时打日志，并交 SafeController 做 DEM 上报 === */
        auto check_limb_voltage = [this](float v, int& state, const char* name) {
            int cur = (v < 54.0f) ? kVfUnder : (v > 90.0f) ? kVfOver : kVfNone;
            if (cur != state) {
                if (cur == kVfUnder) {
                    logger_.warn("%s UNDERVOLTAGE: %.2fV < 54V", name, v);
                } else if (cur == kVfOver) {
                    logger_.error("%s OVERVOLTAGE: %.2fV > 90V", name, v);
                } else {
                    const char* fault_name[] = {"", "ZERO", "UNDERVOLTAGE", "OVERVOLTAGE"};
                    logger_.info("%s voltage RECOVERED from %s: %.2fV", name, fault_name[state], v);
                }
                state = cur;
            }
        };
        check_limb_voltage(voltage_status.up_voltage, up_volt_fault_state_, "UpLimb");
        check_limb_voltage(voltage_status.down_voltage, down_volt_fault_state_, "DownLimb");

        // 接线：把主板电压（输入+输出）交给 SafeController 做 DEM 故障上报（内部去重）
        if (auto sc = safe_controller_.lock()) {
            sc->CheckMainboardVoltage(voltage_status.bat_voltage,
                                      voltage_status.up_voltage,
                                      voltage_status.down_voltage);
        }
        return;
    }

    /* 电机通信状态回调：缓存数据 + 状态变化告警 + bus_off告警 + 错误/丢包阈值告警。
     * 1. 使能/标零状态变化时输出告警日志（状态机：每个变化记录一次）
     * 2. bus_off 状态变化时输出告警日志（0->1 异常，1->0 恢复）
     * 3. err_frame_recent 或 pkt_loss_recent >= 50 时输出告警日志
     * 此回调在 ecan_master RunRoutine 接收线程中同步执行，仅做轻量处理。 */
    void handlMotorCommStatus(const MotorCommStatusResponse& status)
    {
        // /* === 电机通信状态调试日志 === */
        // logger_.info("MotorCommStatus callback! count=%d", status.count);
        // for (uint8_t i = 0; i < status.count && i < 8; i++) {
        //     const auto& m = status.motors[i];
        //     logger_.info("  motor[%d] ch=%d id=%d en=%d hm=%d err_tot=%u err_1k=%u loss_tot=%u loss_1k=%u",
        //                  i, m.can_ch, m.can_id, m.enable_status, m.set_home_status,
        //                  m.err_frame_total, m.err_frame_recent, m.pkt_loss_total, m.pkt_loss_recent);
        // }
        // /* === 调试日志结束 === */
        // {
        //     std::lock_guard<std::mutex> lock(motor_comm_mutex_);
        //     latest_motor_comm_ = status;
        // }
        // uint8_t n = status.count;
        // if (n > 128) n = 128;  // 安全截断
        // for (uint8_t i = 0; i < n; i++) {
        //     const auto& m = status.motors[i];
        //     // 按 (can_ch, can_id) 定位，支持增量上报
        //     uint16_t key = ((uint16_t)m.can_ch << 4) | (m.can_id & 0x0F);

        //     /* === 使能/标零状态变化告警 === */
        //     {
        //         if (motor_comm_prev_[key].enable_status != m.enable_status) {
        //             logger_.warn("MotorCommStatus ch=%u id=%u: enable changed %u->%u",
        //                          m.can_ch, m.can_id,
        //                          motor_comm_prev_[key].enable_status, m.enable_status);
        //         }
        //         if (motor_comm_prev_[key].set_home_status != m.set_home_status) {
        //             logger_.warn("MotorCommStatus ch=%u id=%u: set_home changed %u->%u",
        //                          m.can_ch, m.can_id,
        //                          motor_comm_prev_[key].set_home_status, m.set_home_status);
        //         }
        //         /* === bus_off 状态变化告警（CAN总线异常关闭/恢复） === */
        //         if (motor_comm_prev_[key].bus_off != m.bus_off) {
        //             if (m.bus_off) {
        //                 logger_.error("MotorCommStatus ch=%u id=%u: CAN BUS-OFF! bus_off changed 0->1",
        //                               m.can_ch, m.can_id);
        //             } else {
        //                 logger_.warn("MotorCommStatus ch=%u id=%u: CAN bus-off recovered, bus_off changed 1->0",
        //                              m.can_ch, m.can_id);
        //             }
        //         }
        //     }
        //     motor_comm_prev_[key].enable_status = m.enable_status;
        //     motor_comm_prev_[key].set_home_status = m.set_home_status;
        //     motor_comm_prev_[key].bus_off = m.bus_off;

        //     /* === 错误帧/丢包阈值告警 === */
        //     if (m.err_frame_recent >= 50 || m.pkt_loss_recent >= 50) {
        //         logger_.warn("MotorCommStatus ch=%u id=%u: enable=%u set_home=%u "
        //                      "err_frame(total=%u recent=%u) pkt_loss(total=%u recent=%u)",
        //                      m.can_ch, m.can_id, m.enable_status, m.set_home_status,
        //                      m.err_frame_total, m.err_frame_recent,
        //                      m.pkt_loss_total, m.pkt_loss_recent);
        //     }
        // }

        // // 接线：把电机通信状态交给 SafeController 做 DEM 故障上报（内部去重，状态变化才发布）
        // if (auto sc = safe_controller_.lock()) {
        //     sc->CheckMotorCommStatus(status);
        // }
    }

    // 轮询硬急停恢复状态：电机数据帧刷新足够多后触发 OnHardStopRecoveredAllJoints
    void CheckHardStopRecoverPending() {
        if (!hstop_recover_pending_.load(std::memory_order_acquire)) return;
        if (hstop_status) return; // 若再次进入硬停则等待下一次释放set_home_status
        // 等待至少 kHstopRecoverDataFrames 帧电机数据刷新，确保关节数据就绪
        if (run_cnt_ - hstop_snapshot_run_cnt_ < kHstopRecoverDataFrames) return;
        hstop_recover_pending_.store(false, std::memory_order_release);
        if (hstop_recover_thread_.joinable()) hstop_recover_thread_.join();
        hstop_recover_thread_ = std::thread([this]() {
            logger_.info("Hard stop recovered (%d frames elapsed), invoking OnHardStopRecoveredAllJoints",
                         kHstopRecoverDataFrames);
            OnHardStopRecoveredAllJoints();
        });
    }
    virtual void JointsNumInit()
    {
        std::string model = GetRobotModel();
    
        ecan_wd_ = new Watchdog(1000*1000*100, std::bind(&EcanHardwareInterface::CallBack_, this,std::placeholders::_1,
                     std::placeholders::_2, std::placeholders::_3));
        ecan_motor = std::make_shared<Motor>("eth1", 96, 6);
        BaseActor::StartAllActors(0);
        //等待线程启动
        sleep(2);
        ecan_motor->MotorEnable(model);
        // Register joint raw data callback
        ecan_motor->registerCallback<JointRawData>("motor", std::bind(&EcanHardwareInterface::handleMotorData,  this, std::placeholders::_1));
        ecan_motor->registerCallback<ECAN_STATUS>("ecanstatus", std::bind(&EcanHardwareInterface::handlEcanStatus,  this, std::placeholders::_1));
        ecan_motor->registerCallback<VoltageStatus>("voltage", std::bind(&EcanHardwareInterface::handlVoltageStatus, this, std::placeholders::_1));
        ecan_motor->registerCallback<MotorCommStatusResponse>("motorcomm", std::bind(&EcanHardwareInterface::handlMotorCommStatus, this, std::placeholders::_1));
    }
    bool IsDataUpdatedFinished(){
        bool res = data_updated_[0];
        for(int i=0; i < dof_num_; ++i){
            res = res & data_updated_[i];
        }
        return res;
    }
    void ResetJointError()
    {
        // CR1 关节排布顺序：腰 + 左臂 + 右臂 + 左腿 + 右腿 + 头(neck)
        const int waist_start = 0;
        const int waist_end   = waist_start + waist_dof_num_;
        const int arm_start   = waist_end;
        const int arm_end     = arm_start + 2 * arm_dof_num_;
        const int leg_start   = arm_end;
        const int leg_end     = leg_start + 2 * leg_dof_num_;
        const int neck_start  = leg_end;
        const int neck_end    = neck_start + neck_dof_num_;

        // 按 头 -> 臂 -> 腿 -> 腰 的顺序分组清错
        struct Group { const char* name; int start; int end; };
        const Group groups[] = {
            {"head",  neck_start,  neck_end },
            {"arm",   arm_start,   arm_end  },
            {"leg",   leg_start,   leg_end  },
            {"waist", waist_start, waist_end},
        };

        // 使能前暂停安全控制，避免使能瞬间关节状态异常触发软急停
        auto sc = safe_controller_.lock();
        const bool sc_was_paused = sc ? sc->isPaused() : true;
        if (sc && !sc_was_paused) {
            sc->setPaused(true);
            logger_.info("ResetJointError: safe controller paused before enable phase");
        }
        
        const uint16_t control_word[] = {kIndexDisable, kIndexErrorReset, kIndexEnable, kIndexGetStatusWord};

        auto& data_container = joints_cmd.data();
        for (const auto& grp : groups) {
            if (grp.end <= grp.start) continue;  // 该分组不存在(例如标准版无头部)则跳过
            for (int j = 0; j < 4; ++j) {
                {
                    std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
                    for (int i = 0; i < dof_num_; ++i) {
                        auto& joint_data = data_container.at(i);
                        if (i >= grp.start && i < grp.end) {
                            joint_data.control_word(control_word[j]);
                        } else {
                            // 非当前分组关节仅查询状态字，避免被联动切换状态
                            joint_data.control_word(kIndexGetStatusWord);
                        }
                    }
                    HandlerJointCmd(&joints_cmd);
                }
                usleep(10 * 1000);
            }
        }


        // 对全部关节连续下发 Enable 指令 200ms（每 5ms 一帧，共 40 帧），
        // 固化使能状态，避免部分驱动器漏使能
        for (int k = 0; k < 40; ++k) {
            {
                std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
                for (int i = 0; i < dof_num_; ++i) {
                    data_container.at(i).control_word(kIndexEnable);
                }
                HandlerJointCmd(&joints_cmd);
            }
            usleep(5 * 1000);
        }

        // 使能完成后等待 1s，确保驱动器稳定后再恢复安全控制判断
        if (sc && !sc_was_paused) {
            usleep(1000 * 1000);
            sc->setPaused(false);
            logger_.info("ResetJointError: safe controller resumed after enable phase");
        }
    }
    
    void CallBack_(int timeoutsSinceStart, int timeoutsSinceFeed, double elapsedTime) {
    logger_.warn("DDS_CMD topic data update timeout %f", elapsedTime);
    if(elapsedTime < 2.0){
      JointRawData data_send;
      // Map the received DDS data to the motor joint data structure
      for (int i = 0; i < kJointCount; i++) {
        data_send[i].joint_data.tor = 0;
        data_send[i].joint_data.pos = 0;
        data_send[i].joint_data.vel = 0;
        data_send[i].joint_data.kp  = 0;
        data_send[i].joint_data.kd  = 10;
        data_send[i].control_word = kIndexMotorControl;
      }
      ecan_motor->SetJointsCMD(data_send);
    }else if(elapsedTime < 2.5){
      JointRawData data_send;
      // Map the received DDS data to the motor joint data structure
      for (int i = 0; i < kJointCount; i++) {
        data_send[i].joint_data.tor = 0;
        data_send[i].joint_data.pos = 0;
        data_send[i].joint_data.vel = 0;
        data_send[i].joint_data.kp  = 0;
        data_send[i].joint_data.kd  = 0;
        data_send[i].control_word = kIndexDisable;
      }
      ecan_motor->SetJointsCMD(data_send);
      logger_.warn("DDS_CMD topic data update timeout %f. Disable lynx", elapsedTime);
    }else
    {
      JointRawData data_send;
      // Map the received DDS data to the motor joint data structure
      for (int i = 0; i < kJointCount; i++) {
        data_send[i].joint_data.tor = 0;
        data_send[i].joint_data.pos = 0;
        data_send[i].joint_data.vel = 0;
        data_send[i].joint_data.kp  = 0;
        data_send[i].joint_data.kd  = 0;
        data_send[i].control_word = kIndexMotorControl;
      }
      ecan_motor->SetJointsCMD(data_send);
      ecan_wd_->Stop();
      logger_.warn("DDS_CMD topic data update timeout %f. GetStatus", elapsedTime);
    }
}

void HandlerJointCmd(const drdds::msg::JointsCmd *data)
{
    std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
    JointRawData RobotCMD = {0};
    // Map the received DDS data to the motor joint data structure
    for (size_t i = 0; i < dof_num_; i++) {
      joints_cmd.data().at(i) = data->data().at(i);
      RobotCMD[i].joint_data.tor = data->data().at(i).torque();
      RobotCMD[i].joint_data.pos = data->data().at(i).position();  // Placeholder: Set to appropriate joint position
      RobotCMD[i].joint_data.vel = data->data().at(i).velocity();  // Placeholder: Set to appropriate joint velocity
      RobotCMD[i].joint_data.kp  = data->data().at(i).kp() * 10;   // Placeholder: Set to appropriate joint P gain
      RobotCMD[i].joint_data.kd  = data->data().at(i).kd() * 10;   // Placeholder: Set to appropriate joint D gain
      RobotCMD[i].control_word = data->data().at(i).control_word();
      if( RobotCMD[i].control_word == kIndexMotorControl) ecan_wd_->Start();
     // printf("ledcmd %lu, pos %f ver %f \r\n ", i, data->data().at(i).position(),data->data().at(i).velocity());
    }
    ecan_wd_->Feed();
    ecan_motor->SetJointsCMD(RobotCMD);
    snapshot_.DDSJointCmdHandler(&joints_cmd, dof_num_);
}

void handleMotorDataCheck(const JointRawData& data)
{
    static JointRawData last_data = {0};
    uint8_t num = 0;
    static uint64_t data_no_up_num[kJointCount] = {0};
    static uint64_t data_no_up_nummax[kJointCount] = {0};
    static uint64_t data_no_up_numtotal[kJointCount] = {0};
    static uint64_t data_no_up_num_all = 0;
    static uint64_t data_no_up_num_max = 0;
    static uint64_t data_no_up_num_all_total = 0;

    for (size_t i = 0; i < dof_num_; i++) 
    {
       if(last_data[i].joint_data.pos == data[i].joint_data.pos 
        && last_data[i].joint_data.vel == data[i].joint_data.vel
        && last_data[i].joint_data.tor == data[i].joint_data.tor
        &&  !hstop_status)
        {
            // 未携带 neck 关节硬件时，neck 段（i >= dof_num_ - neck_dof_num_）数据由假数据填充，
            // 跳过数据不更新校验，避免误报
            if (!with_neck_ && neck_dof_num_ > 0 &&
                static_cast<int>(i) >= dof_num_ - neck_dof_num_) {
                continue;
            }
            num++;
            data_no_up_num[i]++;
            data_no_up_numtotal[i]++;

            if(data_no_up_num[i] > data_no_up_nummax[i])
            {
                data_no_up_nummax[i] = data_no_up_num[i];
                if(data_no_up_nummax[i] < 10)
                {
                    //不打印
                }
                else if(data_no_up_nummax[i] < 100)
                {
                    if(0 == data_no_up_nummax[i] % 10)
                    logger_.warn("关节 %d数据不更新次数超过%u,请检查关节接线情况!",i, data_no_up_nummax[i]);
                }
                else if(data_no_up_nummax[i] < 1000)
                {
                    if(0 == data_no_up_nummax[i] % 100)
                    logger_.warn("关节 %d数据不更新次数超过%u,请检查关节是否正常!",i, data_no_up_nummax[i]);
                }
                else if(data_no_up_nummax[i] < 10000)
                {
                    if(0 == data_no_up_nummax[i] % 1000)
                    logger_.error("关节 %d数据不更新次数超过%u,请检查关节是否在线!",i, data_no_up_nummax[i]);
                }
                else
                {
                    //不打印
                }
            }
        }
        else
        {
            data_no_up_num[i] = 0;
            data_no_up_nummax[i] = 0;
        }
        last_data[i].joint_data.pos = data[i].joint_data.pos; 
        last_data[i].joint_data.vel = data[i].joint_data.vel;
        last_data[i].joint_data.tor = data[i].joint_data.tor;
    }
}
 /**
   * @brief Handles the joint raw data callback.
   * 
   * This function processes the received joint data and sends it to the DDS topic.
   * It updates each joint's position, velocity, torque, and temperature data.
   * 
   * @param data The joint data received from the Motor class.
   */
  void handleMotorData(const JointRawData& data) {
   //  std::cout << "Joint pos : >>> " << data[2].joint_data.pos << std::endl;
    struct timespec timestamp;
    static double last_time = 0;
    clock_gettime(CLOCK_MONOTONIC, &timestamp);
    double recv_time = timestamp.tv_sec + double(timestamp.tv_nsec)/1.e9;
    if((recv_time - last_time) > 0.005){
      logger_.warn("handleLegMotorData timeout, delta time is %+1.5f \n", recv_time - last_time);
    }
    last_time = timestamp.tv_sec + double(timestamp.tv_nsec)/1.e9;

    for (size_t i = 0; i < dof_num_; i++) 
    {
      if(0 != data[i].board_info.board_id)
      {

          joints_data.data()[i].position() = data[i].joint_data.pos;
          joints_data.data()[i].velocity() = data[i].joint_data.vel;
          joints_data.data()[i].torque() = data[i].joint_data.tor;
          joints_data.data()[i].motion_temp() = data[i].joint_data.motor_temp / 100.0f;
          joints_data.data()[i].driver_temp() = data[i].joint_data.driver_temp / 100.0f;
          joints_data.data()[i].status_word() = data[i].status_word;
          joints_data.header().timestamp().sec() = timestamp.tv_sec;
          joints_data.header().timestamp().nsec() = timestamp.tv_nsec;
      }
      // 未携带 neck 关节硬件时，对 neck 段下发假数据（参考原 head 假数据逻辑），避免下游因状态字异常误报
      if (!with_neck_ && neck_dof_num_ > 0 &&
          static_cast<int>(i) >= dof_num_ - neck_dof_num_) {
          joints_data.data()[i].position()    = data[0].joint_data.pos / 100;
          joints_data.data()[i].status_word() = 1;
      }
    }
    // 硬急停恢复期跳过数据更新检查，避免虚假告警和计数器误抬高
    if (!hstop_recover_pending_.load(std::memory_order_acquire)) {
        handleMotorDataCheck(data);
    }
    Handler(&joints_data);
    snapshot_.DDSJointRawDataHandler(&joints_data, dof_num_);
}


void DataPubRunner() {
    uint64_t cnt = 0;
    struct timespec system_time_;

    // 创建 timerfd，1ms 周期
    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    struct itimerspec time_intv;
    time_intv.it_value.tv_sec = 0;
    time_intv.it_value.tv_nsec = 1000 * 1000;  // 1ms
    time_intv.it_interval.tv_sec = 0;
    time_intv.it_interval.tv_nsec = 1000 * 1000;  // 1ms
    timerfd_settime(tfd, 0, &time_intv, NULL);

    // 创建 epoll
    int efd = epoll_create1(0);
    struct epoll_event ev;
    struct epoll_event evs[1];
    ev.data.fd = tfd;
    ev.events = EPOLLIN;
    epoll_ctl(efd, EPOLL_CTL_ADD, tfd, &ev);

    while (start_flag_) {
        int fds = epoll_wait(efd, evs, 1, -1);
        if (fds > 0 && (evs[0].events & EPOLLIN)) {
            uint64_t value;
            read(evs[0].data.fd, &value, sizeof(uint64_t));
        }
        cnt++;
        clock_gettime(CLOCK_REALTIME, &system_time_);
        ch_joints_data->data_->header().timestamp().sec() = system_time_.tv_sec;
        ch_joints_data->data_->header().timestamp().nsec() = system_time_.tv_nsec;
        // 未携带 neck 关节硬件时，neck 段（i >= dof_num_ - neck_dof_num_）话题上报位置默认为 0
        const int neck_skip_start = (!with_neck_ && neck_dof_num_ > 0)
                                        ? (dof_num_ - neck_dof_num_)
                                        : dof_num_;
        {
            std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
            for(int i = 0; i < dof_num_; i++)
            {
                ch_joints_data->data_->data().at(i).position() = (i >= neck_skip_start) ? 0.0f : joint_pos_(i);
                ch_joints_data->data_->data().at(i).velocity() = joint_vel_(i);
                ch_joints_data->data_->data().at(i).torque() = joint_tau_(i);
                ch_joints_data->data_->data().at(i).motion_temp() = motor_temperture_(i);
                ch_joints_data->data_->data().at(i).driver_temp() = driver_temperture_(i);
                ch_joints_data->data_->data().at(i).status_word() = driver_status_[i];
            }
        }
        // ch_joints_data->Write();  // 高频发布已移至 SDK 状态
        if(cnt%100 == 0)
        {
            ch_joints_data_10Hz->data_->data() =  ch_joints_data->data_->data();
            ch_joints_data_10Hz->Write();
        }
        if(cnt%1000 == 0)
        {
            hes_status_sub_->Write();
        }
        /* === 临时测试：每5秒主动查询一次电机通信状态 === */
        if(cnt % 5000 == 0 && cnt > 0)
        {
            ecan_motor->RequestMotorCommStatus();
        }
        /* === 临时测试结束 === */
        // 硬急停恢复检测（在1ms周期里轮询，开销极小）
        CheckHardStopRecoverPending();
    }

    close(tfd);
    close(efd);
}


void hmap_write(const VecXf& new_data) {
    std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
    hmap_is_updated = true;
    VecXf& target = hmap_buffers_[hmap_write_index_];
    target = new_data;  // 写入非活跃缓冲区
    hmap_write_index_.store(1 - hmap_write_index_.load());  // 切换缓冲区
}

void HeightsSubCallback(const sensor_msgs::msg::Image* input_cloud_msg) {
    static double last_hmap_time_stamp=0.0;
    timespec start_timestamp, end_timestamp;
    clock_gettime(CLOCK_MONOTONIC,&start_timestamp);

    if (!input_cloud_msg) {
        std::cerr << "[ERROR] Null message received!" << std::endl;
        return;
    }
    const uint32_t sec = input_cloud_msg->header().stamp().sec();
    const int32_t nanosec = input_cloud_msg->header().stamp().nanosec();
    const double current_time_stamp_ms = sec*1000.0 + static_cast<double>(nanosec) / 1e6;
    double delta_time_ms=current_time_stamp_ms-last_hmap_time_stamp;
    last_hmap_time_stamp=current_time_stamp_ms;
    const std::string m_frame_id=input_cloud_msg->header().frame_id();
    const uint32_t msg_width = input_cloud_msg->width();
    const uint32_t msg_height = input_cloud_msg->height();
    const std::string& encoding = input_cloud_msg->encoding();
    const uint32_t msg_step = input_cloud_msg->step();
    const std::vector<uint8_t>& data = input_cloud_msg->data();
    
    static int cout_cnt=0;
    cout_cnt++;
    cout_cnt=cout_cnt>1e4?0:cout_cnt;

    if (data.empty()) {
        std::cerr << "[ERROR] Empty image data!" << std::endl;
        return;
    }
    if (encoding != "32FC1") {
        std::cerr << "[ERROR] Unsupported encoding: " << encoding 
                << " (expected 32FC1)" << std::endl;
        return;
    }
    const size_t expected_bytes = msg_height * msg_step;
    if (data.size() < expected_bytes) {
        std::cerr << "[ERROR] Data size mismatch! Expected at least " 
                << expected_bytes << " bytes, got " 
                << data.size() << std::endl;
        return;
    }

    int nan_count = 0, inf_count = 0, out_limit = 0;

    const int step = static_cast<int>(msg_step);
    
    Eigen::MatrixXf hmap_mat;
    hmap_mat.resize(x_points_num, y_points_num);
    hmap_mat.setZero();
    // 按行解析数据
    for (int row = 0; row < x_points_num; ++row) {//原始高度图34*21，策略输入34*21
        // 每行起始指针（按 step 对齐）
        const float* row_data = reinterpret_cast<const float*>(&data[row * step]);
        // 复制当前行数据到 Eigen
        for (int col = 0; col < y_points_num; ++col) {
            float val = row_data[col];
            // 检查有效性（仅统计，不逐像素打印，每200帧汇总一次）
            if (std::isnan(val) ){
                val = 0.0f;
                nan_count++;
            } 
            else if (std::isinf(val)) {
                val = (val > 0) ? body_height: -body_height;
                inf_count++;
            }
            else if (std::abs(val ) >2.0) {
                val = (val > 0) ? 2.0: -2.0; 
                out_limit++;
            }
            // float value_temp=-(val+body_height);//地面是0
            float value_temp=val;//取均值版本
            // if(value_temp>0.18&&row<20){
            //     value_temp=0.6;
            // }
            // if(value_temp>0.18&&row>20){
            // //  value_temp=0.0;
            // }

            // hmap_mat(row,col)=value_temp; // sim和实际高度以及方向差异
            // sim_hmap_sequence(row*y_points_num+col) = value_temp;
            sim_hmap_sequence((x_points_num-1-row)*y_points_num + (y_points_num-1-col)) = value_temp;
            // std::cout<<"val:"<<val<<std::endl;
            // std::cout<<"value_temp:"<<value_temp<<std::endl;
        }
    }
    // 每200帧汇总打印一次高度图异常统计，避免逐像素刷屏
    if (cout_cnt % 200 == 0 && (nan_count > 0 || inf_count > 0 || out_limit > 0)) {
        logger_.warn("Hmap anomalies: NaN=%d Inf=%d out_limit=%d", nan_count, inf_count, out_limit);
    }

    Eigen::VectorXf centered = sim_hmap_sequence.array() - sim_hmap_sequence.mean();//取均值版本

    // hmap_mat.row(x_points_num-1)=hmap_mat.row(x_points_num-2);// sim和实际hmap size 差异
    // std::cout<<"sim_hmap_sequence:"<<sim_hmap_sequence.transpose()<<std::endl;

    // for(int i=0;i<y_points_num;i++){
    //     for(int j=0;j<x_points_num;j++){
    //         sim_hmap_sequence(i*x_points_num+j)=hmap_mat(x_points_num-1-j,y_points_num-1-i);//旋转塞入
    //     }
    // }
    // std::cout<<"get hmap!"<<std::endl;
    // std::cout<<"hmap_mat:"<<hmap_mat<<std::endl;
    // std::cout<<"centered:"<<centered.transpose()<<std::endl;

    //update_hmap_vec(sim_hmap_sequence);  // 线程安全更新
    hmap_write(centered);
}

public:
    int run_cnt_ =0;    
    Snapshot snapshot_;
    EcanHardwareInterface(const std::string& robot_name, const int &dof_num, const int &leg_dof_num, const int &arm_dof_num, const int &waist_dof_num, const int &neck_dof_num)
    :RobotInterface(robot_name, dof_num, leg_dof_num, arm_dof_num, waist_dof_num, neck_dof_num), snapshot_(10),data_recorder_(10),voltage_recorder_(10),logger_("ecan"){
        std::cout <<"init dof:"<<leg_dof_num_<<", "<<arm_dof_num_<<", "<<waist_dof_num_<<", "<<neck_dof_num_<<", "<<dof_num_<< std::endl;
        std::cout << robot_name << " is using Ecan Hardware Interface" << std::endl;
        // 加载 [ecan_hardware] 配置（包含 with_neck 开关）
        LoadEcanHardwareConfig();
        joint_pos_ = VecXf::Zero(dof_num_);
        joint_pos_raw_ = VecXf::Zero(dof_num_);
        joint_vel_ = VecXf::Zero(dof_num_);
        joint_tau_ = VecXf::Zero(dof_num_);
        motor_temperture_ = VecXf::Zero(dof_num_);
        driver_temperture_ = VecXf::Zero(dof_num_);

        pos_offset_ = new float[dof_num_];
        joint_dir_ = new float[dof_num_];
        data_updated_ = new bool[dof_num_];
        joint_config_ = new JointConfig[dof_num_];

        driver_status_.resize(dof_num_);
        joint_data_id_.resize(dof_num_);
        battery_data_.resize(BATTERY_DATA_SIZE);
        joints_data.data().resize(dof_num_);
        joints_cmd.data().resize(dof_num_);
        sim_hmap_sequence.setZero(x_points_num * y_points_num);

        data_recorder_.SetBatteryData(&snapshot_.GetBatteryInfo());
        data_recorder_.SetJointRawData(&snapshot_.getJointVector());
        voltage_recorder_.SetVoltageData(&latest_voltage_, &voltage_mutex_);

        ch_imu_head_data_ = std::make_unique<ChannelImu>("/IMU_DATA_HEAD", 0);
        ch_imu_base_data_ = std::make_unique<ChannelImu>("/IMU_DATA_BASE", 0);
        ch_imu_data_ = std::make_unique<ChannelImu>(std::bind(&EcanHardwareInterface::HandlerIMUBody, this, std::placeholders::_1),"/IMU_DATA_BODY", 0);
        ch_imu_leg_data_ = std::make_unique<ChannelImuData>(std::bind(&EcanHardwareInterface::HandlerIMULeg, this, std::placeholders::_1),"/IMU_DATA_LEG", 0);
        
        h_battery_data_ = std::make_unique<ChannelBatteryData>(std::bind(&EcanHardwareInterface::HandlerBattery, this, std::placeholders::_1),"/BATTERY_DATA", 0);
        hes_status_sub_ = std::make_unique<ChannelStdMsgInt32>("/HES_STATUS", 0);
        //接受hmap img
        hmap_sub_ = std::make_unique<ChannelImage>(std::bind(&EcanHardwareInterface::HeightsSubCallback, this, std::placeholders::_1), "/elevation_map_img", 0);
        ch_joints_data = std::make_unique<ChannelJoints>("/JOINTS_DATA", 0);  ///< DDS channel for joint data transmission.
        ch_joints_data_10Hz = std::make_unique<ChannelJoints>("/JOINTS_DATA_10HZ", 0);  ///< DDS channel for joint data transmission.

        ch_joints_data->data_->data().resize(dof_num_);  
        ch_joints_data_10Hz->data_->data().resize(dof_num_);


        //初始化ecan
        JointsNumInit();
        sleep(1);
        ResetJointError();
        start_flag_ = true;
        data_pub_thread_ = std::thread(std::bind(&EcanHardwareInterface::DataPubRunner, this));

    }
    virtual ~EcanHardwareInterface(){
        start_flag_ = false;
        // 通知硬急停恢复线程中的busy-loop尽快退出，避免析构卡住
        stop_recover_.store(true, std::memory_order_release);
        if (data_pub_thread_.joinable()) {//确保线程在对象销毁时能够正确退出
            data_pub_thread_.join();
        }
        if (hstop_recover_thread_.joinable()) {
            hstop_recover_thread_.join();
        }
        // 先断开所有DDS订阅，避免回调在对象销毁过程中继续访问已释放资源
        ch_imu_data_.reset();
        ch_imu_leg_data_.reset();
        ch_imu_head_data_.reset();
        ch_imu_base_data_.reset();
        h_battery_data_.reset();
        hes_status_sub_.reset();
        hmap_sub_.reset();
        delete [] pos_offset_;
        delete [] joint_dir_;
        delete [] data_updated_;
        delete [] joint_config_;
    }

    virtual void Start(){
    }

    virtual void Stop(){
    }
    virtual void InitPositionOffset(){}

    // 硬急停恢复后、所有关节数据都更新完毕时触发的回调，派生类按需 override
    virtual void OnHardStopRecoveredAllJoints() {}
    virtual double GetInterfaceTimeStamp(){
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return ri_ts_;
    }
    virtual VecXf GetJointPosition() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return joint_pos_;
    }
    virtual VecXf GetJointPositionRaw() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return joint_pos_raw_;
    }
    
    virtual VecXf GetJointVelocity() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return joint_vel_;
    }
    virtual VecXf GetJointTorque() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return joint_tau_;
    }
    virtual Vec3f GetImuBodyRpy() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return rpy_body_;
    }
    virtual Vec3f GetImuBodyAcc() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return acc_body_;
    }
    virtual Vec3f GetImuBodyOmega() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return omega_body_;
    }
    virtual Vec3f GetImuRpy() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return rpy_leg_;
    }
    virtual Vec3f GetImuAcc() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return acc_leg_;
    }
    virtual Vec3f GetImuOmega() {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return omega_leg_;
    }
    virtual VecXf GetContactForce() {
        return VecXf::Zero(4);
    }
    virtual VecXf GetHeightMapVec()
    {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return hmap_buffers_[1 - hmap_write_index_.load()];;
    }
    virtual void SetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input){
        // 硬急停恢复期间，外部主控指令一律走damping，避免与恢复线程下发的0指令争抢ecan_motor
        if (hstop_recover_pending_.load(std::memory_order_acquire)) {
            sendDampingCommand();
            return;
        }
        DoSetJointCommand(input);
    }

    // 恢复线程专用入口：不走pending门禁，供 ResetPositionOffset 下发0指令使用
    void DoSetJointCommand(Eigen::Matrix<float, Eigen::Dynamic, 5> input){
        std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
        // 脖子接管覆盖（默认关闭零代价）
        ApplyNeckTakeover(input);

        // === NaN/Inf 检测 ===
        if (input.hasNaN()) {
            logger_.error("SetJointCommand: NaN detected in input, sending damping command!");
            sendDampingCommand();
            return;
        }
        if (!input.allFinite()) {
            logger_.error("SetJointCommand: Inf detected in input, sending damping command!");
            sendDampingCommand();
            return;
        }
        // === 三级保护检查 ===
        if (joint_protection_.isEnabled()) {
            auto result = joint_protection_.check(input);
            if (result == ProtectionResult::L1_FATAL) {
                // L1触发：丢弃指令，发送阻尼保护命令
                //sendDampingCommand();
                return;
            }
            if (result == ProtectionResult::L2_ERROR) {
                // L2连续超限触发：发送阻尼保护命令
                //sendDampingCommand();
                return;
            }
        }
        // === 原有逻辑不变 ===
        auto& data_container = joints_cmd.data();
        joint_cmd_ = input;
        for (int i = 0; i < dof_num_; ++i) {
            auto& joint_data = data_container.at(i);
            joint_data.position() = (input(i, 1) - joint_config_[i].offset) * joint_config_[i].dir;
            joint_data.velocity() = input(i, 3) * joint_dir_[i];
            joint_data.torque() = input(i, 4) * joint_dir_[i];
            joint_data.kp(input(i, 0));
            joint_data.kd(input(i, 2));
            joint_data.control_word(kIndexMotorControl);
        }
        HandlerJointCmd(&joints_cmd);
    }
    virtual void SetJointZero(uint64_t jointMask)
    {
        std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
        auto& data_container = joints_cmd.data();
        for (int i = 0; i < dof_num_; ++i) {
            if(jointMask & (1 << i))
            {
                auto& joint_data = data_container.at(i);
                joint_data.control_word(kIndexSetHome);
            }
        }
        HandlerJointCmd(&joints_cmd);
        //只发一次SetHome
        for (int i = 0; i < dof_num_; ++i) {
            auto& joint_data = data_container.at(i);
            joint_data.control_word(kIndexMotorControl);
        }
        // 设置各部位关节命令
    }

    virtual VecXf GetMotorTemperture(){
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return motor_temperture_;
    }
    virtual VecXf GetDriverTemperture(){
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return driver_temperture_;
    }
    virtual double GetImuTimestamp(){
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return imu_leg_ts_;
    }
    virtual double GetImuBodyTimestamp(){
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return imu_body_ts_;
    }
    virtual std::vector<uint16_t> GetBatteryData(){
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return battery_data_;
    }
    virtual BatteryInfo_dds* GetBatteryInfo(){
        // Keep the legacy pointer API while returning a read-only caller snapshot.
        thread_local BatteryInfo_dds battery_snapshot[2];
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        battery_snapshot[0] = battery_info_[0];
        battery_snapshot[1] = battery_info_[1];
        return battery_snapshot;
    }
    MatXf GetJointCommand() override {
        std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
        return joint_cmd_;
    }

    virtual std::vector<uint16_t> GetDriverStatusWord(){
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return driver_status_;
    }
    virtual std::vector<uint16_t> GetJointDataID(){
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return joint_data_id_;
    }
    virtual void RefreshRobotData(){
    }
    virtual void WriteSnapShot()
    {
        snapshot_.Write();
    }
    void SetSafeControllerSource(std::shared_ptr<SafeController> sc) {
        safe_controller_ = sc;
    }

    void initJointProtection(const ControlParameters* cp_ptr, FaultClient* fault_client = nullptr) {
        std::lock_guard<std::recursive_mutex> command_lock(command_mutex_);
        std::string config_path;
        for (const char* candidate : {
                 "/var/opt/robot/conf/motion_master/config.toml",
                 "conf/config.toml"}) {
            if (std::filesystem::exists(candidate)) {
                config_path = candidate;
                break;
            }
        }
        if (!config_path.empty()) {
            try {
                auto toml_config = toml::parse(config_path);
                joint_protection_.init(dof_num_, cp_ptr, toml_config, fault_client);
                logger_.info("JointProtection initialized from %s", config_path.c_str());
            } catch (const std::exception& e) {
                logger_.error("Failed to init JointProtection: %s", e.what());
            }
        } else {
            logger_.warn("JointProtection config not found, protection disabled");
        }
    }
    virtual bool GetHmapUpdateFlag()
    {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        return hmap_is_updated;
    }
    void Handler(const drdds::msg::Joints* data) {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        ++run_cnt_;
        for(int i=0;i<dof_num_;++i){
            joint_pos_(i) = data->data().at(i).position()*joint_dir_[i] + pos_offset_[i] / 180. * M_PI;
            joint_pos_raw_(i) = data->data().at(i).position()*joint_dir_[i];
            joint_vel_(i) = data->data().at(i).velocity()*joint_dir_[i];
            joint_tau_(i) = data->data().at(i).torque()*joint_dir_[i];
            motor_temperture_(i) = float(data->data().at(i).motion_temp());
            driver_temperture_(i) = float(data->data().at(i).driver_temp());
            driver_status_[i] = data->data().at(i).status_word();
            joint_data_id_[i] = uint16_t(run_cnt_);
        }
        ri_ts_ = GetTimestampMs()/1000.;
    }

    void HandlerIMULeg(const drdds::msg::ImuData* data) {
        if (!data) return;
        const double roll = Deg2Rad(data->data().roll());
        const double pitch = Deg2Rad(data->data().pitch());
        const double yaw = Deg2Rad(data->data().yaw());
        {
            std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
            rpy_leg_ = Vec3f(roll, pitch, yaw);
            acc_leg_ << data->data().acc_x(), data->data().acc_y(), data->data().acc_z();
            omega_leg_ << Vec3f(data->data().omega_x(), data->data().omega_y(), data->data().omega_z());
            imu_leg_ts_ = GetTimestampMs();
        }
        snapshot_.DDSLegImuDataHandler(data);

        Eigen::Quaterniond base_quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                                       Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                       Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
        base_quat.normalize();

        sensor_msgs::msg::Imu base_data;
        base_data.header().stamp().sec() = data->header().timestamp().sec();
        base_data.header().stamp().nanosec() = data->header().timestamp().nsec();
        base_data.orientation().x() = base_quat.x();
        base_data.orientation().y() = base_quat.y();
        base_data.orientation().z() = base_quat.z();
        base_data.orientation().w() = base_quat.w();
        base_data.angular_velocity().x() = data->data().omega_x();
        base_data.angular_velocity().y() = data->data().omega_y();
        base_data.angular_velocity().z() = data->data().omega_z();
        base_data.linear_acceleration().x() = data->data().acc_x();
        base_data.linear_acceleration().y() = data->data().acc_y();
        base_data.linear_acceleration().z() = data->data().acc_z();
        ch_imu_base_data_->Write(&base_data);
    }

    void HandlerIMUBody(const sensor_msgs::msg::Imu* data) {
        if (!data) return;
        ch_imu_head_data_->Write(data);
        // 从四元数反算 RPY（ZYX 顺序，弧度）
        const double qx = data->orientation().x();
        const double qy = data->orientation().y();
        const double qz = data->orientation().z();
        const double qw = data->orientation().w();
        const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
        const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
        const double roll = std::atan2(sinr_cosp, cosr_cosp);
        const double sinp = 2.0 * (qw * qy - qz * qx);
        const double pitch = std::abs(sinp) >= 1.0 ? std::copysign(M_PI / 2.0, sinp) : std::asin(sinp);
        const double siny_cosp = 2.0 * (qw * qz + qx * qy);
        const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);
        {
            std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
            rpy_body_ = Vec3f(static_cast<float>(roll), static_cast<float>(pitch), static_cast<float>(yaw));
            acc_body_ << data->linear_acceleration().x(), data->linear_acceleration().y(), data->linear_acceleration().z();
            omega_body_ << Vec3f(data->angular_velocity().x(), data->angular_velocity().y(), data->angular_velocity().z());
            imu_body_ts_ = GetTimestampMs();
        }
        snapshot_.DDSArmImuDataHandler(data);
    }

    void HandlerBattery(const drdds::msg::BatteryData* data) {
        std::lock_guard<std::recursive_mutex> feedback_lock(feedback_mutex_);
        battery_info_[1].battery_level = uint8_t(data->data().at(0).battery_level());        // 电量
        battery_info_[1].voltage = uint16_t(data->data().at(0).voltage());            //电压 
        battery_info_[1].current = uint16_t(data->data().at(0).current());            //电流//现在电池上报有点问题 只有正值
        battery_info_[1].protected_state = uint16_t(data->data().at(0).protected_state()); //故障报警
         for(size_t j = 0; j < data->data().at(0).battery_temperature().size(); j++){
          battery_info_[1].battery_temperature[j] =  data->data().at(0).battery_temperature().at(j);
        }
        battery_data_[0] = (data->data().at(0).battery_level());        // 电量
        battery_data_[1] = (data->data().at(0).voltage());        // 电压
        battery_data_[2] = (data->data().at(0).current());        // 电压
        battery_data_[3] = (data->data().at(0).protected_state());        // 电压
    }
};





#endif
