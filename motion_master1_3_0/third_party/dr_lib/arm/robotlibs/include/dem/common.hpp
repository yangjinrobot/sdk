
#ifndef DEM_COMMON_HPP
#define DEM_COMMON_HPP

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "drdds/core/common_type.h"

// ============================== 导出宏定义 ============================
#if defined(_WIN32)
#ifdef DEM_EXPORTS
#define DEM_API __declspec(dllexport)
#else
#define DEM_API __declspec(dllimport)
#endif
#else
#define DEM_API __attribute__((visibility("default")))
#endif

// ================================ 枚举定义 ============================
// Severity levels (higher value = higher severity)
// 注：因旧版 DRDDS 仅含 WARN/ERROR/FATAL，新增级别以字面值定义
enum Severity {
  DEBUG = 0,
  INFO = 1,
  NOTICE = 2,
  WARN = drdds::msg::FaultEvent_Constants::SEVERITY_WARN,      // = 3
  ERROR = drdds::msg::FaultEvent_Constants::SEVERITY_ERROR,     // = 4
  CRITICAL = drdds::msg::FaultEvent_Constants::SEVERITY_FATAL,  // = 5
  FATAL = CRITICAL,  ///< 兼容性别名, 等同于 CRITICAL
  ALERT = 6,
  EMERG = 7
};

enum FaultType {
  START = drdds::msg::FaultEvent_Constants::TYPE_START,
  STOP = drdds::msg::FaultEvent_Constants::TYPE_STOP,
};

enum ThresholdType { GREATER_IS_WORSE, LESS_IS_WORSE };

// ================================= 常量定义===========================
inline constexpr const char* FAULT_TYPE_UNKNOWN = "unknown";
inline constexpr const char* FAULT_TYPE_START = "start";  // 故障开始
inline constexpr const char* FAULT_TYPE_STOP = "stop";    // 故障结束

inline constexpr const char* FAULT_SEVERITY_UNKNOWN = "unknown";
inline constexpr const char* FAULT_SEVERITY_DEBUG = "debug";
inline constexpr const char* FAULT_SEVERITY_INFO = "info";
inline constexpr const char* FAULT_SEVERITY_NOTICE = "notice";
inline constexpr const char* FAULT_SEVERITY_WARN = "warn";
inline constexpr const char* FAULT_SEVERITY_ERROR = "error";
inline constexpr const char* FAULT_SEVERITY_CRITICAL = "critical";
inline constexpr const char* FAULT_SEVERITY_FATAL = "fatal";  ///< 兼容性别名, 等同于 critical
inline constexpr const char* FAULT_SEVERITY_ALERT = "alert";
inline constexpr const char* FAULT_SEVERITY_EMERG = "emerg";

// ======================= 故障Code定义 ================================
// 说明：故障代码枚举 注意：此定义需与 fault_rules.toml 保持一致
//
// === Hardware Faults (0x8000-0x8FFF) ===
// # 0x80xx : Joint actuators (motors, encoders, drivers, communication)
// # 0x81xx : Battery & power (cells, BMS, charging, short circuit)
// # 0x82xx : Perception sensors (IMU, LiDAR, camera, RTK)
// # 0x83xx : Main control & computing (CPU, memory, storage, temperature, process)
// # 0x84xx : Communication peripherals (buttons, hard estop, 5V ctrl)
//
// === Software Faults (0x9000-0x9FFF) ===
// # 0x90xx : Motion control & gait
// # 0x91xx : Autonomous navigation
// # 0x92xx : Localization & mapping
// # 0x93xx : Perception algorithms
// # 0x94xx : Autonomous behavior (charging)
// # 0x95xx : System runtime (heartbeat, time, network, node runtime)
enum FaultGroup {
  // Hardware Faults
  JOINT_ACTUATORS = 0x8000,
  BATTERY_POWER = 0x8100,
  PERCEPTION_SENSORS = 0x8200,
  MAIN_CONTROL = 0x8300,
  COMM_PERIPHERALS = 0x8400,
  LOAD_PROTECTION = 0x8500,
  // Software Faults
  MOTION_CONTROL = 0x9000,
  AUTONOMOUS_NAV = 0x9100,
  LOCALIZATION = 0x9200,
  PERCEPTION_ALGO = 0x9300,
  AUTONOMOUS_BEHAVIOR = 0x9400,
  SYSTEM_RUNTIME = 0x9500
};

enum FaultCode {
  // -------------------- 0x80xx Joint actuators --------------------
  MOTOR_OVER_TEMP = 0x8001,          // motor_over_temp
  JNT_DRV_OVER_TEMP = 0x8002,        // joint_driver_over_temp
  JNT_DRV_OVER_VOLT = 0x8003,        // joint_driver_over_voltage
  JNT_DRV_UNDER_VOLT = 0x8004,       // joint_driver_under_voltage
  JNT_DRV_OVER_CURR = 0x8005,        // joint_driver_over_current
  JNT_DRV_TIMEOUT = 0x8006,          // joint_driver_timeout
  JNT_DRV_STATUS_ERR = 0x8007,       // joint_driver_status_error
  JNT_DRV_EXCP = 0x8008,             // joint_driver_exception
  JNT_ENC_NO_VAL = 0x8009,           // joint_encoder_no_value
  JNT_HIGH_SPEED = 0x800A,           // joint_continuous_high_speed
  JNT_ENC_OFFLINE = 0x800B,          // joint_encoder_offline
  ENC_CALIB_FAIL = 0x800E,           // encoder_calibration_failed
  TEMP_SENSOR_DISCON = 0x8012,       // temperature_sensor_disconnected
  JNT_VEL_OVER_LIM = 0x8013,         // joint_velocity_over_limit
  JNT_POS_OVER_LIM = 0x8014,         // joint_position_over_limit
  JNT_SUSP_EXCP = 0x8015,            // joint_suspension_exception
  JNT_DATA_NAN = 0x8016,             // joint_data_nan
  JNT_DATA_UPD_ERR = 0x8017,         // joint_data_update_error
  ENC_CHECK_FAIL = 0x800C,           // encoder_check_failed
  ENC_MULTITURN_LOST = 0x800D,       // encoder_multiturn_lost
  JNT_COMM_TIMEOUT = 0x800F,         // joint_comm_timeout
  JNT_COMM_DISCON = 0x8010,          // joint_comm_disconnected
  JNT_COMM_UNSTABLE = 0x8011,        // joint_comm_unstable
  JNT_DRV_DISABLED = 0x8018,         // joint_driver_disabled
  JNT_EXCP_PROT = 0x8019,            // joint_exception_protection
  JNT_HOMING_FAIL = 0x801A,          // joint_homing_failed
  JNT_DRV_DATA_TIMELINESS = 0x801B,  // joint_driver_data_timeliness
  JNT_DRV_TIME_JITTER = 0x801C,      // joint_driver_time_jitter
  JNT_DRV_FW_ERROR = 0x801D,         // joint_driver_firmware_error
  JNT_DRV_DATA_QUALITY = 0x801E,     // joint_driver_data_quality
  JNT_ERR_FRAME_EXCESS = 0x801F,     // joint_error_frame_excessive
  JNT_LOST_FRAME_EXCESS = 0x8020,    // joint_lost_frame_excessive
  JNT_BUS_OFF = 0x8021,              // joint_bus_off
  JNT_CMD_ERROR = 0x8022,            // joint_command_error

  // -------------------- 0x81xx Battery & power --------------------
  BAT_LOW_SOC = 0x8101,             // battery_low_soc
  BAT_LOW_VOLT = 0x8102,            // battery_low_voltage
  BAT_SOC_SURGE = 0x8103,           // battery_soc_sudden_change
  BAT_CELL_UNDER_VOLT = 0x8104,     // battery_cell_under_voltage
  BAT_CELL_OVER_VOLT = 0x8105,      // battery_cell_over_voltage
  BAT_PACK_OVER_VOLT = 0x8106,      // battery_pack_over_voltage
  BAT_PACK_UNDER_VOLT = 0x8107,     // battery_pack_under_voltage
  BAT_DISCH_OVER_TEMP = 0x8108,     // battery_discharge_over_temp
  BAT_DISCH_UNDER_TEMP = 0x8109,    // battery_discharge_under_temp
  BAT_DISCH_OVER_CURR = 0x810A,     // battery_discharge_over_current
  BAT_CHG_OVER_TEMP = 0x810B,       // battery_charge_over_temp (已废弃)
  BAT_CHG_UNDER_TEMP = 0x810C,      // battery_charge_under_temp (已废弃)
  BAT_CHG_OVER_CURR = 0x810D,       // battery_charge_over_current
  BAT_SHORT_CIRCUIT = 0x810E,       // battery_short_circuit
  BAT_IC_ERROR = 0x810F,            // battery_ic_error
  BAT_MOS_LOCK = 0x8110,            // battery_mos_lock
  BAT_DATA_COLL_FAIL = 0x8111,      // battery_data_collection_failed
  BAT_OUT_ZERO_CURR = 0x8112,       // battery_output_zero_current
  BAT_REM_CAP_ERROR = 0x8113,       // battery_remaining_capacity_error
  BAT_BMS_CHECK_FAIL = 0x8114,      // battery_bms_data_check_failed
  BAT_BMS_DATA_TIMEOUT = 0x8115,    // battery_bms_data_timeout
  BAT_CHG_MOS_BREAKDOWN = 0x8116,   // battery_charge_mos_breakdown (GF bit13)
  BAT_DHG_MOS_BREAKDOWN = 0x8117,   // battery_discharge_mos_breakdown (GF bit14)
  BAT_POWER_OVER_TEMP = 0x8118,     // battery_power_over_temp (GF bit15)
  BAT_AFE_COMM_FAIL = 0x8119,       // battery_afe_comm_fail (XWD R110 bit0)
  BAT_CHG_OVER_VOLT = 0x811A,       // battery_charge_over_voltage (XWD R110 bit1)
  BAT_DHG_SECOND_OCP = 0x811B,      // battery_discharge_second_ocp (XWD R110 bit9)
  BAT_FUSE_OPEN = 0x811C,           // battery_fuse_open (XWD R110 bit12)
  BAT_BALANCE_OTP = 0x811D,         // battery_balance_otp (XWD R110 bit13)
  BAT_MOS_OVER_TEMP = 0x811E,       // battery_mos_over_temp (XWD R110 bit14)
  BAT_CHG_MOS_FAIL = 0x811F,        // battery_charge_mos_fail (XWD R110 bit15)
  BAT_DHG_MOS_FAIL = 0x8120,        // battery_discharge_mos_fail (XWD R111 bit0->16)
  BAT_CELL_TEMP_ABNORMAL = 0x8121,  // battery_cell_temp_abnormal (XWD R111 bit1->17)
  BAT_CELL_IMBALANCE = 0x8122,      // battery_cell_imbalance (XWD R111 bit2->18)
  BAT_NTC_WIRE_FAULT = 0x8123,      // battery_ntc_wire_fault (XWD R111 bit3->19)
  BAT_PRECHARGE_PROT = 0x8124,      // battery_precharge_protection (XWD R111 bit4->20)
  BAT_WATER_LEAK = 0x8125,          // battery_water_leak (XWD R111 bit5->21)
  BAT_MULTI_DHG_PROT = 0x8126,      // battery_multi_discharge_protection (XWD R111 bit6->22)
  BAT_CELL_LOW_NOCHG = 0x8127,      // battery_cell_low_no_charge (XWD R111 bit7->23)

  // -------------------- 0x82xx Perception sensors --------------------
  IMU_PUB_TIMEOUT = 0x8201,                 // imu_publish_timeout
  IMU_OPEN_FAIL = 0x8202,                   // imu_open_failed
  IMU_TIMEOUT = 0x8203,                     // imu_data_timeout
  IMU_NAN = 0x8204,                         // imu_data_nan
  LIDAR_EXCP = 0x8205,                      // lidar_exception
  IMU_DATA_CHECK_ERROR = 0x8206,            // imu_data_check_error
  LIDAR_DATA_SRC_LOST = 0x8207,             // lidar_data_source_lost
  RTK_POS_INACCURATE = 0x8208,              // rtk_position_inaccurate
  CAM_STATUS_ERR = 0x8209,                  // camera_status_error
  STREAMING_ERROR = 0x820A,                 // streaming_error
  MIC_ERROR = 0x820B,                       // microphone_error
  UWB_ERROR = 0x820C,                       // uwb_error
  LIDAR_DATA_TIMELINESS = 0x8210,           // lidar_data_timeliness
  LIDAR_TIME_JITTER = 0x8211,               // lidar_time_jitter
  LIDAR_FW_ERROR = 0x8212,                  // lidar_firmware_error
  LIDAR_DATA_QUALITY = 0x8213,              // lidar_data_quality
  LIDAR_TIME_SYNC_STATE_ABNORMAL = 0x8214,  // lidar_time_sync_state_abnormal
  IMU_DATA_TIMELINESS = 0x8220,             // imu_data_timeliness
  IMU_TIME_JITTER = 0x8221,                 // imu_time_jitter
  IMU_FW_ERROR = 0x8222,                    // imu_firmware_error
  IMU_DATA_QUALITY = 0x8223,                // imu_data_quality
  RTK_DATA_TIMELINESS = 0x8230,             // rtk_gps_data_timeliness
  RTK_TIME_JITTER = 0x8231,                 // rtk_gps_time_jitter
  RTK_FW_ERROR = 0x8232,                    // rtk_gps_firmware_error
  RTK_DATA_QUALITY = 0x8233,                // rtk_gps_data_quality
  CAM_DATA_TIMELINESS = 0x8240,             // camera_data_timeliness
  CAM_TIME_JITTER = 0x8241,                 // camera_time_jitter
  CAM_FW_ERROR = 0x8242,                    // camera_firmware_error
  CAM_DATA_QUALITY = 0x8243,                // camera_data_quality

  // -------------------- 0x83xx Main control & computing --------------------
  CPU_TEMP_HIGH = 0x8301,          // cpu_temp_high
  CPU_USAGE_HIGH = 0x8302,         // cpu_usage_high
  CPU_FREQ_LOW = 0x8303,           // cpu_freq_low
  MEM_AVAILABLE_LOW = 0x8304,      // memory_available_low
  MEM_TEMP_HIGH = 0x8305,          // memory_temp_high
  FS_USAGE_HIGH = 0x8306,          // fs_usage_high
  DISK_TEMP_HIGH = 0x8307,         // disk_temp_high
  FS_INODE_HIGH = 0x8308,          // fs_inode_usage_high
  PROC_FD_LEAK = 0x8309,           // proc_fd_leak
  PROC_THREAD_LEAK = 0x830A,       // proc_thread_leak
  PROC_MEMORY_HIGH = 0x830B,       // proc_memory_high
  PROC_PAGE_FAULT_STORM = 0x830C,  // proc_page_fault_storm
  PROC_CTX_SWITCH_STORM = 0x830D,  // proc_ctx_switch_storm
  PROC_IO_STORM = 0x830E,          // proc_io_storm
  CPU_IOWAIT_HIGH = 0x830F,        // cpu_iowait_high
  MEM_OOM_KILL = 0x8310,           // mem_oom_kill
  PROC_ZOMBIE_HIGH = 0x8311,       // proc_zombie_high
  MB_INPUT_OVER_VOLT = 0x8312,     // mainboard_input_over_voltage
  MB_INPUT_UNDER_VOLT = 0x8313,    // mainboard_input_under_voltage
  MB_OUTPUT_OVER_VOLT = 0x8314,    // mainboard_output_over_voltage
  MB_OUTPUT_UNDER_VOLT = 0x8315,   // mainboard_output_under_voltage

  // -------------------- 0x84xx Communication peripherals --------------------
  ENERGY_SAVING_SW_ERR = 0x8401,  // energy_saving_switch_error
  SOFT_ESTOP_BTN_ERR = 0x8402,    // soft_estop_button_error
  HARD_ESTOP = 0x8403,            // hard_estop
  CTRL_5V_ERROR = 0x8404,         // ctrl_5v_error
  SPEAKER_ERROR = 0x8405,         // speaker_error
  DISPLAY_ERROR = 0x8406,         // display_error

  // -------------------- 0x850x Load protection --------------------
  LOAD_OVERLOAD_PROT = 0x8501,  // load_overload_protection

  // -------------------- 0x90xx Motion control & gait --------------------
  SOFT_ESTOP_RECV = 0x9001,            // soft_estop_received
  GAIT_SWITCH_FAIL = 0x9002,           // gait_switch_failed
  FALL_DETECTED = 0x9003,              // fall_detected
  ACTION_EXEC_FAIL = 0x9004,           // action_exec_failed
  PROLONGED_IDLE_WALK = 0x9005,        // prolonged_idle_walking
  PROLONGED_STANDING = 0x9006,         // prolonged_standing
  BODY_POSE_ERR = 0x9007,              // body_pose_error
  MOTION_POSE_ERR = 0x9008,            // motion_pose_error
  MOTION_FALL_ESTOP = 0x9009,          // motion_fall_estop
  LIE_TO_STAND_FAIL = 0x900A,          // lie_to_stand_failed
  STEP_TO_STOP_FAIL = 0x900B,          // step_to_stop_failed
  STAND_TO_FORCE_FAIL = 0x900C,        // stand_to_force_ctrl_failed
  FORCE_CTRL_TO_STEP_FAIL = 0x900D,    // force_ctrl_to_step_failed
  NAV_MODE_SWITCH_FAIL = 0x900E,       // nav_mode_switch_failed
  UPRIGHT_CRAWL_SWITCH_FAIL = 0x900F,  // upright_crawl_switch_failed
  IDLE_WALK_TIMEOUT = 0x9010,          // idle_walk_timeout

  // -------------------- 0x91xx Autonomous navigation --------------------
  NAV_PROC_NOT_STARTED = 0x9101,   // nav_process_not_started
  NAV_MODULE_STATUS_ERR = 0x9102,  // nav_module_status_error
  PERSISTENT_OBSTACLE = 0x9103,    // persistent_obstacle_stop
  NAV_PLAN_FAIL = 0x9104,          // nav_plan_failed
  NAV_VEL_NO_UPDATE = 0x9105,      // nav_velocity_no_update
  NAV_ARRIVAL_TIMEOUT = 0x9106,    // nav_arrival_timeout
  NAV_TASK_CANCELLED = 0x9107,     // nav_task_cancelled
  NAV_TASK_CONFLICT = 0x9108,      // nav_task_conflict
  NAV_SEND_GOAL_FAIL = 0x9109,     // nav_send_goal_failed
  NAV_CANCEL_FAIL = 0x910A,        // nav_cancel_failed
  NAV_PARAM_SET_FAIL = 0x910B,     // nav_param_set_failed
  NAV_MODULE_RESTART = 0x910C,     // nav_module_restart
  NAV_GOAL_IN_OBSTACLE = 0x910D,   // nav_goal_in_obstacle

  // -------------------- 0x92xx Localization & mapping --------------------
  LOCALIZATION_FAIL = 0x9201,       // localization_failed
  FOOT_ODOM_ABNORMAL = 0x9208,      // foot_odometry_abnormal
  MAP_LOAD_FAIL = 0x9202,           // map_load_failed
  MAP_AUTO_UPDATE_FAIL = 0x9203,    // map_auto_update_failed
  MAP_SWITCH_FAIL = 0x9204,         // map_switch_failed
  LOC_MODULE_RESTART = 0x9205,      // localization_module_restart
  SENSOR_ERROR = 0x9206,            // sensor_error
  ODOM_PUB_TIME_ERROR = 0x9207,     // odom_publish_time_error
  SET_TERRAIN_PARAM_FAIL = 0x9209,  // set_terrain_param_failed
  NO_LOCATION_DATA = 0x920A,        // no_location_data

  // -------------------- 0x93xx Perception algorithms --------------------
  POINTCLOUD_ABNORMAL = 0x9304,        // pointcloud_abnormal
  TERRAIN_DATA_TIMEOUT = 0x9301,       // terrain_data_timeout
  TERRAIN_DATA_INVALID = 0x9302,       // terrain_data_invalid
  TERRAIN_MODULE_STATUS_ERR = 0x9303,  // terrain_module_status_error
  LIDAR_ODOM_START_FAIL = 0x9305,      // lidar_odom_start_failed
  LIDAR_ODOM_STOP_FAIL = 0x9306,       // lidar_odom_stop_failed
  NO_POINTCLOUD_DATA = 0x9309,         // no_pointcloud_data

  // -------------------- 0x94xx Autonomous behavior (charging) --------------------
  CHARGE_TAG_TIMEOUT = 0x9401,       // charge_get_tag_data_timeout
  CHARGE_ALGO_FAIL = 0x9402,         // charge_algo_launch_failed
  CHARGE_NO_TAG = 0x9403,            // charge_no_tag_data
  CHARGE_TAG_JITTER = 0x9404,        // charge_tag_data_jitter
  CHARGE_PILE_DISCON = 0x9405,       // charge_pile_disconnected
  CHARGE_NO_CURR = 0x9406,           // charge_no_current_detected
  CHARGE_APPROACH_OBST = 0x9407,     // charge_approach_obstacle
  CHARGE_DEPART_OBST = 0x9408,       // charge_departure_obstacle
  CHARGE_ARRIVAL_TMO = 0x9409,       // charge_arrival_timeout
  CHARGE_DEPART_TMO = 0x940A,        // charge_departure_timeout
  CHARGE_EXIT_FAIL = 0x940B,         // charge_exit_failed
  CHARGE_ENTER_FAIL = 0x940C,        // charge_enter_failed
  CHARGE_HAND_CHARGING = 0x940D,     // charge_hand_charging_detected
  CHARGE_TASK_ERROR = 0x940E,        // charge_task_error
  CHARGE_PROC_RESTART = 0x940F,      // charge_process_restart
  CHARGE_MANUAL_CHARGING = 0x9410,   // charge_manual_charging_no_move
  CHARGE_MOTION_STATE_ERR = 0x9411,  // charge_motion_state_error

  // -------------------- 0x95xx System runtime --------------------
  HB_TIMEOUT = 0x9500,             // heartbeat_timeout (统一心跳超时码)
  NET_TX_HIGH = 0x9503,            // network_tx_high
  NET_RX_HIGH = 0x9504,            // network_rx_high
  NET_LINK_DOWN = 0x9505,          // network_link_down
  TIME_SYNC_ERROR = 0x9501,        // time_sync_error
  SYS_TIME_JITTER = 0x9502,        // system_time_jitter
  NODE_INIT_FAIL = 0x9506,         // node_init_failed
  PARAM_CHECK_FAIL = 0x9507,       // param_check_failed
  SYNC_CB_TIMEOUT = 0x9508,        // sync_callback_timeout
  DATA_PUB_FAIL = 0x9509,          // data_publish_failed
  RUNTIME_UNRECOVER_ERR = 0x950A,  // runtime_unrecoverable_error
  ODOM_NO_UPDATE = 0x950B,         // odom_no_update
  CONFIG_FILE_MISSING = 0x950C,    // config_file_missing
  TCP_CLOSE_WAIT_HIGH = 0x950D,    // tcp_close_wait_high
  TCP_RETRANS_HIGH = 0x950E,       // tcp_retrans_high
};

// =============================== 故障事件结构定义 ==========================
struct FaultEventInfo {
  FaultType fault_type;
  Severity new_severity;  // for START
  Severity old_severity;  // for STOP
};

// ======================== 条件值与故障条件结构 ========================

/// 条件值：severity 等级对应的策略参数
struct ConditionValue {
  Severity severity;  ///< DEBUG / INFO / NOTICE / WARN / ERROR / CRITICAL / ALERT / EMERG
  double value;       ///< 策略参数值，语义由 strategy 决定
};

/// 统一故障检测条件结构体，兼容所有策略类型
struct FaultCondition {
  std::string strategy;  ///< 策略标签: thresholds/bit_match/timeout/count/state_match/immediate
  ///<  (type以@拼接在末尾，如 bit_match@GF)
  std::optional<ThresholdType> direction;  ///< thresholds 专用: GREATER_IS_WORSE/LESS_IS_WORSE
  std::optional<double> hysteresis;        ///< thresholds 专用: 滞回值
  uint32_t confirm_count = 1;              ///< 通用消抖次数
  std::vector<std::string> except;         ///< 通用排除列表
  std::vector<ConditionValue> values;      ///< 核心值列表
};

// ======================== 故障配置项结构 ========================
/**
 * @brief 单条故障配置项（格式无关）
 * @details getConfig() 返回此结构，conditions 为强类型条件列表
 */
struct FaultCfgItem {
  uint32_t code = 0;
  std::string name;
  Severity severity = Severity::WARN;
  std::vector<FaultCondition> conditions;  ///< 强类型条件列表
};

// ======================== LevelMonitor 配置结构 ========================
/**
 * @brief LevelMonitor 的阈值配置（格式无关）
 * @details 从 FaultCondition 提取的阈值专用配置
 */
struct LevelMonitorConfig {
  ThresholdType direction = GREATER_IS_WORSE;
  std::vector<std::pair<Severity, double>> thresholds;
  double hysteresis = 0.0;
  uint32_t confirm_count = 1;
};

/**
 * @brief 从强类型条件列表中提取 LevelMonitor 配置
 * @param conditions 强类型条件列表
 * @return 解析成功返回 LevelMonitorConfig，失败返回 std::nullopt
 * @details 遍历条件列表，找到第一个 strategy=="thresholds" 的条件并转换
 */
DEM_API std::optional<LevelMonitorConfig> parseThresholdCondition(const std::vector<FaultCondition>& conditions);

// ======================== 转换函数 ========================
DEM_API std::optional<Severity> toSeverity(const std::string& level_name);
DEM_API std::string toString(Severity severity);
DEM_API std::string toString(FaultType fault_type);

/**
 * @brief 将 std::chrono::system_clock::time_point 转换为
 * builtin_interfaces::msg::Time
 * @param tp 系统时钟时间点
 * @return 转换后的 builtin_interfaces::msg::Time 对象
 */
DEM_API builtin_interfaces::msg::Time timePointToMsg(const std::chrono::system_clock::time_point& tp);

// ======================== 辅助函数声明 ========================
std::vector<uint8_t> toSeverityVec(Severity severity);
DEM_API std::vector<uint8_t> toSeverityVec(const std::string& severity);
DEM_API std::string getSocId();

/**
 * @brief 判断是否是心跳超时故障码
 * @param code 故障码
 * @return true 如果 code 属于 SYSTEM_RUNTIME 分组（0x95xx）
 */
inline bool isHbTimeout(uint32_t code) { return (code & 0xFF00) == static_cast<uint32_t>(SYSTEM_RUNTIME); }

// ============ 辅助函数 ============

/**
 * @brief 获取默认的 severities 数组（单元素）
 * @param severity 严重程度等级
 * @return 包含单个严重程度值的数组
 */
inline std::vector<uint8_t> toSeverityVec(Severity severity) { return {static_cast<uint8_t>(severity)}; }

/**
 * @brief 解析 source 字符串，提取进程名称和实例 ID
 * @param source 原始 source 字符串，格式为 "process_name#instance_id" 或 "process_name"
 * @return std::pair<process_name, instance_id>
 *         - process_name: 进程名称（不含实例标识）
 *         - instance_id: 实例标识（如 soc_id），若无则为空字符串
 * @details
 * 使用 '#' 作为分隔符，避免与进程名中常见的下划线冲突。
 * 支持的格式：
 *   - "motion_master#soc0" -> ("motion_master", "soc0")
 *   - "motion_master#0" -> ("motion_master", "0")
 *   - "motion_master" -> ("motion_master", "")
 */
DEM_API std::pair<std::string, std::string> parseSource(const std::string& source);

#endif  // DEM_COMMON_HPP
