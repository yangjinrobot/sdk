/**
 * @file safe_controller.hpp
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

#ifndef SAFE_CONTROLLER_HPP_
#define SAFE_CONTROLLER_HPP_

#include "robot_interface.h"
#include "user_command_interface.h"
#include "dr_logger.hpp"
#include "drdds/core/drdds_core.h"
#include "data_recorder.h"
#include "dem/fault_client.hpp"
#include "dem/level_monitor.hpp"
#include "parameters/control_parameters.h"
#include "safe_controller/temperature_monitor.hpp"
#include "safe_controller/joint_monitor.hpp"
#include "types/data_structures.h"

#include <atomic>
#include <cmath>
#include <cstdio>
#include <map>
#include <mutex>
#include <optional>
#include <unordered_map>



union RobotErrorState{ 
    struct{ 
        uint32_t imu_num_error : 1;        // bit0: IMU数据为NaN
        uint32_t imu_update_overtime: 1;   // bit1: IMU数据更新超时(20ms)
        uint32_t heatbeat_overtime : 1;    // bit2: 关节通讯心跳超时
        uint32_t driver_error: 1;          // bit3: 驱动器状态错误
        uint32_t motor_heat_warn : 1;      // bit4: 电机过温告警
        uint32_t joint_num_error : 1;      // bit5: 编码器无值/关节数据异常
        uint32_t joint_update_error : 1;   // bit6: 关节数据不更新
        uint32_t battery_low_warn : 1;     // bit7: 低电量/低电压告警
        uint32_t battery_heat_warn : 1;    // bit8: 电池过温告警
        uint32_t cpu_heat_warn : 1;        // bit9: CPU过温告警
        uint32_t cpu_freq_warn : 1;        // bit10: CPU频率异常告警
        uint32_t driver_heat_warn : 1;     // bit11: 驱动器过温告警
        uint32_t joint_pos_error : 1;      // bit12: 关节角超限
        uint32_t joint_nan_error : 1;      // bit13: 关节数据为NaN
        uint32_t joint_vel_error : 1;      // bit14: 关节角速度超限
        uint32_t posture_error : 1;        // bit15: 身体姿态异常(roll/pitch超限)
        uint32_t reserved : 16;            // bit16-31: 预留
    }; 
    uint32_t error_code; 
}; 

// 故障分组枚举（优先级：LEG > WAIST_XY > IMU_BODY > HEAD > UPPER_LIMB > WAIST_Z）
enum class BodyFaultGroup { NONE, HEAD, UPPER_LIMB, WAIST_Z, WAIST_XY, LEG, IMU_BODY, IMU_HEAD };

class SafeController
{
private:
    RobotType robot_type_;
    // 生命周期内不变，Start() 前完成 SetXxxDataSource 初始化，无 data race 风险
    std::shared_ptr<interface::RobotInterface> ri_ptr_;
    std::shared_ptr<interface::UserCommandInterface> uc_ptr_;
    std::shared_ptr<ControlParameters> cp_ptr_;
    std::thread judge_thread_;
    int run_cnt_ = 0; 

    double last_imu_ts_, imu_check_time_;
    bool imu_check_flag_ = true;

    Vec3f l_leg_joint_pos_, l_leg_joint_vel_, l_leg_joint_tau_;
    Vec3f r_leg_joint_pos_, r_leg_joint_vel_, r_leg_joint_tau_;
    double l_leg_leave_grd_init_time_, r_leg_leave_grd_init_time_;
    bool l_leg_contact_state_, r_leg_contact_state_;

    double ri_ts_ = 0;// / 1000.;
    VecXf joint_pos, joint_vel ,joint_tau;
    VecXf joint_pos_lower, joint_pos_upper;

    // ── 子检测器 ──
    JointMonitor joint_monitor_;
    JointMonitorConfig joint_monitor_cfg_;
    Vec3f last_rpy  , last_acc ,last_omg;
    int imu_data_same_cnt_ = 0;
    int imu_timeout_cnt_   = 0;   // 连续超时帧数，达到阈值才确认故障
    int imu_normal_cnt_    = 0;   // 连续正常帧数，达到阈值才解除故障
    static constexpr int kImuTimeoutConfirmFrames = 5;  // 超时确认帧数（≈5ms@1kHz）
    static constexpr int kImuNormalConfirmFrames  = 3;  // 恢复确认帧数

    double last_cmd_ts_, cmd_check_time_;
    bool cmd_check_flag_ = true;

    types::UserCommand* usr_cmd_;
    bool start_thread_flag_ = false;

    double current_time_;

    RobotErrorState robot_error_state_;
    uint32_t last_error_code_;
    std::map<int, uint16_t> error_driver_map_;
    DrLogger safe_logger_;
    // ── 跨线程共享的故障掩码（Run 线程写入，GetFaultSnapshot/GetFaultMask 读取）──
    std::atomic<uint32_t> joint_update_state_{0};
    std::atomic<uint32_t> joint_pos_error_code{0};
    std::atomic<uint32_t> joint_nan_code{0};
    std::atomic<uint32_t> driver_status_error_code{0};
    std::atomic<uint32_t> imu_update_timeout_flag_{0};
    std::atomic<uint32_t> imu_data_nan_flag_{0};
    // robot_error_state_ 快照（Run 周期末更新，跨线程安全读取）
    std::atomic<uint32_t> snapshot_error_code_{0};
    std::atomic<bool>     snapshot_posture_error_{false};

    uint32_t joint_vel_error_code = 0;                         // single-threaded (Run only)
    uint32_t joint_overheat_notice_code = 0, driver_overheat_notice_code = 0;
    uint32_t joint_overheat_warn_code = 0, driver_overheat_warn_code = 0;
    uint32_t joint_overheat_error_code = 0, driver_overheat_error_code = 0;
    uint32_t joint_overheat_fatal_code = 0, driver_overheat_fatal_code = 0;

    TemperatureMonitor motor_temp_;
    TemperatureMonitor driver_temp_;
    std::optional<LevelMonitorConfig> motor_temp_cfg_;
    std::optional<LevelMonitorConfig> driver_temp_cfg_;

    std::unique_ptr<FaultClient> fault_client_;

    // Unified per-fault-code last-reported state (replaces 20+ individual last_xxx variables)
    struct LastFaultState {
        uint32_t resources = 0;
        std::vector<uint8_t> severities;
    };
    std::unordered_map<int, LastFaultState> last_reported_state_;

    // 姿态超限检测相关
    bool posture_limit_enable_ = true;
    // 仅上报模式：true 时不向 usr_cmd_->safe_control_mode 写入故障位，
    // 避免下游状态机触发软急停（仅用于诊断/调试）。
    bool report_only_ = false;
    std::atomic<bool> paused_{false};  // true 时暂停安全控制判断，强制 safe_control_mode=0
    float roll_threshold_deg_ = 15.0f;
    float pitch_threshold_deg_ = 20.0f;
    int hysteresis_ms_ = 100;
    int posture_exceed_count_ = 0;        // 超限持续计数
    int posture_exceed_threshold_ = 0;    // 触发阈值（根据检测频率和hysteresis_ms计算）
    bool posture_limit_exceeded_ = false; // 超限标志

    // 是否携带 neck（头部）关节硬件（从 config.toml [ecan_hardware] with_neck 读取，默认 true）
    bool with_neck_ = true;

    // 主板电压越限阈值 + 严重等级（由 LoadFaultConfig 从故障配置读取，默认 54V/90V, WARN）
    float voltage_under_threshold_ = 54.0f;
    float voltage_over_threshold_  = 90.0f;
    uint8_t voltage_input_under_severity_ = Severity::WARN;  // MB_INPUT_UNDER_VOLT(0x8313)
    uint8_t voltage_input_over_severity_  = Severity::WARN;  // MB_INPUT_OVER_VOLT(0x8312)
    uint8_t voltage_output_under_severity_ = Severity::WARN; // MB_OUTPUT_UNDER_VOLT(0x8315)
    uint8_t voltage_output_over_severity_  = Severity::WARN; // MB_OUTPUT_OVER_VOLT(0x8314)
    bool report_mainboard_input_under_volt_  = false;  // MB_INPUT_UNDER_VOLT(0x8313)
    bool report_mainboard_input_over_volt_   = false;  // MB_INPUT_OVER_VOLT(0x8312)
    bool report_mainboard_output_under_volt_ = false;  // MB_OUTPUT_UNDER_VOLT(0x8315)
    bool report_mainboard_output_over_volt_  = false;  // MB_OUTPUT_OVER_VOLT(0x8314)

    // 电机通信状态故障严重等级 + 阈值（由 LoadFaultConfig 从故障配置读取）
    bool report_motor_comm_disabled_ = false;                  // JNT_DRV_DISABLED(0x8018)
    uint8_t motor_comm_disabled_severity_ = Severity::ERROR;  // JNT_DRV_DISABLED(0x8018)
    bool report_motor_comm_err_      = false;                  // JNT_ERR_FRAME_EXCESS(0x801F)
    uint8_t motor_comm_err_severity_      = Severity::WARN;   // JNT_ERR_FRAME_EXCESS(0x801F)
    int     motor_comm_err_threshold_     = 100;               // JNT_ERR_FRAME_EXCESS 阈值(每1000帧)
    bool report_motor_comm_loss_     = false;                  // JNT_LOST_FRAME_EXCESS(0x8020)
    uint8_t motor_comm_loss_severity_     = Severity::WARN;   // JNT_LOST_FRAME_EXCESS(0x8020)
    int     motor_comm_loss_threshold_    = 100;               // JNT_LOST_FRAME_EXCESS 阈值(每1000帧)
    bool report_motor_comm_busoff_   = false;                  // JNT_BUS_OFF(0x8021)
    uint8_t motor_comm_busoff_severity_   = Severity::ERROR;  // JNT_BUS_OFF(0x8021)

    // 关节驱动器故障严重等级（由 LoadFaultConfig 从故障配置读取，默认 ERROR）
    uint8_t joint_drv_over_volt_severity_  = Severity::ERROR;  // JNT_DRV_OVER_VOLT(0x8003)
    uint8_t joint_drv_under_volt_severity_ = Severity::ERROR;  // JNT_DRV_UNDER_VOLT(0x8004)
    uint8_t joint_drv_over_curr_severity_  = Severity::ERROR;  // JNT_DRV_OVER_CURR(0x8005)
    uint8_t joint_drv_timeout_severity_    = Severity::ERROR;  // JNT_DRV_TIMEOUT(0x8006)
    uint8_t temp_sensor_discon_severity_   = Severity::ERROR;  // TEMP_SENSOR_DISCON(0x8012)
    uint8_t enc_calib_fail_severity_       = Severity::ERROR;  // ENC_CALIB_FAIL(0x800E)
    uint8_t joint_enc_no_val_severity_     = Severity::ERROR;  // JNT_ENC_NO_VAL(0x8009)
    uint8_t joint_drv_status_err_severity_ = Severity::ERROR;  // JNT_DRV_STATUS_ERR(0x8007)

    // driverstatus bit3/bit4 对应的 over-temp mask（仅用于合并，避免重复上报）
    uint32_t motor_over_temp_mask_from_driverstatus_ = 0;
    uint32_t driver_over_temp_mask_from_driverstatus_ = 0;

    bool report_motor_over_temp_ = false;
    bool report_joint_driver_over_temp_ = false;
    bool report_joint_driver_over_volt_ = false;
    bool report_joint_driver_under_volt_ = false;
    bool report_joint_driver_over_curr_ = false;
    bool report_joint_driver_timeout_ = false;
    bool report_temp_sensor_discon_ = false;
    bool report_enc_calib_fail_ = false;
    bool report_joint_enc_no_val_ = false;
    bool report_joint_driver_status_error_ = false;
    bool report_imu_data_timeout_ = false;
    bool report_imu_data_nan_ = false;
    bool report_joint_data_update_error_ = false;

    // BODY_POSE_ERR 上报开关 + fault yaml 严重级别
    bool report_body_pose_err_ = false;
    uint8_t body_pose_err_fault_severity_ = Severity::FATAL;
    // thresholds are in milliseconds to match GetTimestampMs()
    double imu_update_timeout_threshold_ = 40.0;
    double joint_update_timeout_threshold_ = 30;
    int joint_data_same_cnt_threshold_ = 50;
    // 关节数据不更新时为 true：抑制其他关节相关故障(NaN/位置超限/速度超限/驱动器状态/温度)的上报，
    // 但仍保留各自的软急停触发(safe_control_mode)。
    bool joint_data_not_updated_ = false;

    // Standardized debounced fault config: encapsulates enabled + count_threshold + severity
    struct DebouncedFaultConfig {
        bool enabled = false;
        int count_threshold = 30;
        uint8_t severity = Severity::FATAL;

        bool loadFrom(const std::optional<FaultCfgItem>& fault_item) {
            if (!fault_item.has_value()) return false;
            enabled = true;
            // Use severity from FaultCfgItem
            severity = static_cast<uint8_t>(fault_item->severity);
            // Parse count threshold from conditions (timeout/count strategy)
            for (const auto& cond : fault_item->conditions) {
                if (cond.confirm_count > 1) {
                    count_threshold = static_cast<int>(cond.confirm_count);
                    break;
                }
                // Also try extracting from values (error-level value as threshold)
                for (const auto& cv : cond.values) {
                    if (cv.severity == Severity::ERROR && cv.value > 0) {
                        count_threshold = static_cast<int>(cv.value);
                        break;
                    }
                }
            }
            return true;
        }
    };

    DebouncedFaultConfig vel_over_limit_cfg_;
    DebouncedFaultConfig joint_nan_cfg_;
    DebouncedFaultConfig pos_over_limit_cfg_;
    uint8_t imu_timeout_fault_severity_ = Severity::FATAL;
    uint8_t imu_nan_fault_severity_ = Severity::FATAL;
    uint8_t joint_data_upd_fault_severity_ = Severity::FATAL;

    // driverstatus: bit_match strategy uses "bit" field in yaml
    int bit_JNT_DRV_OVER_VOLT_ = 0;
    int bit_JNT_DRV_UNDER_VOLT_ = 1;
    int bit_JNT_DRV_OVER_CURR_ = 2;
    int bit_MOTOR_OVER_TEMP_ = 3;
    int bit_JNT_DRV_OVER_TEMP_ = 4;
    int bit_JNT_DRV_TIMEOUT_ = 5;
    int bit_TEMP_SENSOR_DISCON_ = 6;
    int bit_enc_calib_fail_ = 7;
    int bit_JNT_ENC_NO_VAL_ = 8;
    int bit_JNT_DRV_STATUS_ERR_ = 9;

    /* ---- 电机通信状态(0x8B)/主板电压 故障上报 ----
     * 故障码使用 dem/common.hpp 官方枚举：
     *   JNT_DRV_DISABLED(0x8018) / JNT_ERR_FRAME_EXCESS(0x801F)
     *   JNT_LOST_FRAME_EXCESS(0x8020) / JNT_BUS_OFF(0x8021)
     * 主板电压：
     *   MB_INPUT_OVER_VOLT(0x8312)  / MB_INPUT_UNDER_VOLT(0x8313)  输入
     *   MB_OUTPUT_OVER_VOLT(0x8314) / MB_OUTPUT_UNDER_VOLT(0x8315) 输出 */

    /* 电机通信状态全量缓存：MCU 增量上报（仅发有变化的电机），
     * 这里按 (can_ch<<4)|can_id 合并每帧数据，基于全量缓存计算故障位图。
     * 只在 ecan 接收线程访问，无需加锁。 */
    struct MotorCommCacheEntry {
        uint8_t  enable_status = 0;
        uint16_t err_recent = 0;
        uint16_t loss_recent = 0;
        uint8_t  bus_off = 0;
    };
    std::unordered_map<uint16_t, MotorCommCacheEntry> motor_comm_cache_;
    std::map<uint16_t, int> motor_comm_joint_map_;  // key=(can_ch<<4)|can_id → joint_index
    bool motor_comm_map_init_ = false;
    void buildMotorCommJointMap();

    /* reportFault 互斥锁：judge_thread_ 与 ecan 接收线程都会调用 reportFault，
     * 保护 last_reported_state_ 与 fault_client_->pubEvent */
    std::mutex fault_report_mutex_;


    /**
 * @brief 报告故障状态变化（仅在资源位图或严重程度变化时发布）
 * @param current_resources 当前资源位图（0 表示故障恢复）
 * @param fault_code 故障代码
 * @param severity 每个故障资源对应的严重程度向量
 * @param grouped 是否分组上报
 * @param resource 当 grouped==false 时的单资源标识
 */
void reportFault(
    uint32_t current_resources,
    int fault_code,
    const std::vector<uint8_t>& severity,
    bool grouped,
    const std::string& resource = "");
    /**
 * @brief 将uint32_t转换为表示置1位位置的字符串向量
 * @param value 32位无符号整数
 * @return 包含所有置1位位置字符串的向量
 */
std::vector<std::string> getSetBitsVector(uint32_t value);

    Vec3f GroundContactDetection(Vec3f pos, Vec3f velocity, Vec3f torque);

    bool IsImuDataNormal();
    // BODY_POSE_ERR(BODY_POSE_ERR) based on GroundContactDetection in specific gaits.
    bool IsGroundContactNormal();
    void CheckPostureLimit();
    void SampleTemperatures();
    void LoadFaultConfig();
    void LoadPostureLimitConfig();

    bool IsMotorTempertureNormal();

    bool IsDriverTempertureNormal();
    bool IsBatteryNormal();
    bool IsUserCommandNormal();
    void PrintRobotErrorState();
    void Run();

public:
    SafeController(RobotType robot_type, const std::string& path);
    ~SafeController(){
        start_thread_flag_ = false;
        if (judge_thread_.joinable()) {
            judge_thread_.join();
        }
    }

    void SetRobotDataSource(std::shared_ptr<interface::RobotInterface> ri);
    void SetUserCommandDataSource(std::shared_ptr<interface::UserCommandInterface> uc);
    void SetControlParamInfaceSource(std::shared_ptr<ControlParameters> cp_ptr);
    void Start();

    /**
     * @brief 运行时暂停/恢复安全控制判断（不影响故障上报日志）
     * @param paused true=暂停（safe_control_mode 强制为 0），false=恢复正常判断
     */
    void setPaused(bool paused) { paused_.store(paused, std::memory_order_release); }
    bool isPaused() const { return paused_.load(std::memory_order_acquire); }
    
    // ── 故障状态快照 ──
    struct FaultSnapshot {
        uint32_t error_code             = 0;
        uint32_t pos_error_mask         = 0;
        uint32_t nan_mask               = 0;
        uint32_t update_mask            = 0;
        uint32_t driver_status_mask     = 0;
        bool     imu_update_timeout     : 1 = false;
        bool     imu_data_nan           : 1 = false;
        bool     posture_error          : 1 = false;
    };
    FaultSnapshot GetFaultSnapshot() const;
    std::map<int, uint16_t> GetDriverErrorMap() const { return error_driver_map_; }
    FaultClient* GetFaultClient() { return fault_client_.get(); }

    // 获取合并的故障掩码（关节位置/NaN/更新/驱动状态错误 + 注入掩码）
    inline uint32_t GetDriverStatusErrorCode(){
        return driver_status_error_code;
    }
    inline bool GetPostureError(){
        return static_cast<bool>(robot_error_state_.posture_error);
    }
    bool IsPostureLimitExceeded() const { return posture_limit_exceeded_; }
    inline std::map<int, uint16_t> GetErrorDriverMap(){
        return error_driver_map_;
    }
    inline bool IsImuUpdateTimeout(){
        return imu_update_timeout_flag_;
    }
    inline bool IsImuDataNan(){
        return imu_data_nan_flag_;
    }

    /* 电机通信状态故障检测与上报。故障码使用 dem/common.hpp 官方枚举：
     *   JNT_DRV_DISABLED(0x8018) / JNT_ERR_FRAME_EXCESS(0x801F)
     *   JNT_LOST_FRAME_EXCESS(0x8020) / JNT_BUS_OFF(0x8021)
     * 由 EcanHardwareInterface::handlMotorCommStatus 回调调用（ecan 接收线程）。
     * MCU 增量上报，内部先合并到全量缓存再计算故障位图；
     * reportFault 去重：位图不变不重复发布，位图归零自动发 STOP。 */
    void CheckMotorCommStatus(const ecan::MotorCommStatusResponse& status);

    /**
     * @brief 主板电压（输入+输出）越限检测与上报
     * @param bat_voltage  主板输入电压（电池电压）
     * @param up_voltage   上肢输出电压
     * @param down_voltage 下肢输出电压
     *
     * 输入电压(bat_voltage)：单一资源
     *   - MB_INPUT_UNDER_VOLT(0x8313) / MB_INPUT_OVER_VOLT(0x8312)
     * 输出电压(up/down)：分组 bitmap，bit0=上肢 bit1=下肢
     *   - MB_OUTPUT_UNDER_VOLT(0x8315) / MB_OUTPUT_OVER_VOLT(0x8314)
     *
     * 由 EcanHardwareInterface::handlVoltageStatus 回调调用（1Hz）。
     */
    void CheckMainboardVoltage(float bat_voltage, float up_voltage, float down_voltage);

    uint32_t GetFaultMask();
    // 故障分组判定
    BodyFaultGroup ClassifyFault(uint32_t fault_mask);

private:
    // 注入故障掩码（独立于硬件检测结果，OR 进 GetFaultMask）
    std::atomic<uint32_t> injected_pos_error_code_{0};
    std::atomic<uint32_t> injected_nan_code_{0};
    std::atomic<uint32_t> injected_update_state_{0};
    std::atomic<uint32_t> injected_driver_status_code_{0};

    // 故障注入 DDS 订阅
    // std::unique_ptr<ChannelString> fault_inject_sub_;  // 调试用，已关闭

    // ===== 故障注入接口（调试/测试用，private） =====
    void SetInjectedFaultMask(uint32_t pos, uint32_t nan, uint32_t upd, uint32_t drv);
    void ClearInjectedFaultMask();
    uint32_t GetInjectedFaultMask() const;
};





#endif