#include "safe_controller/safe_controller.hpp"
#include "toml/toml.hpp"
#include "runtime_params.hpp"
#include <filesystem>

namespace {

void ApplyFaultTopSeverity(const std::optional<FaultCfgItem>& fault_item, uint8_t& out_severity) {
    if (!fault_item.has_value()) return;
    out_severity = static_cast<uint8_t>(fault_item->severity);
}

}  // namespace

void SafeController::LoadFaultConfig(){
    auto getFaultCfg = [&](int code) -> std::optional<FaultCfgItem> { return fault_client_->findConfig(static_cast<uint32_t>(code)); };
    auto getBitFromFault = [&](int code, int fallback_bit) -> int {
        auto item = getFaultCfg(code);
        if (!item.has_value()) return fallback_bit;
        for (const auto& cond : item->conditions) {
            if (cond.strategy == "bit_match" && !cond.values.empty()) {
                int v = static_cast<int>(cond.values[0].value);
                if (v >= 0 && v < 16) return v;
            }
        }
        return fallback_bit;
    };
    safe_logger_.info("Loading fault config by code...");
    report_motor_over_temp_ = false;
    report_joint_driver_over_temp_ = false;
    report_joint_driver_over_volt_ = false;
    report_joint_driver_under_volt_ = false;
    report_joint_driver_over_curr_ = false;
    report_joint_driver_timeout_ = false;
    report_temp_sensor_discon_ = false;
    report_enc_calib_fail_ = false;
    report_joint_enc_no_val_ = false;
    report_joint_driver_status_error_ = false;
    report_imu_data_timeout_ = false;
    report_imu_data_nan_ = false;
    report_joint_data_update_error_ = false;
    report_body_pose_err_ = false;

    auto item = getFaultCfg(JNT_DRV_OVER_TEMP);
    if (item) { report_joint_driver_over_temp_ = true; auto monCfg = parseThresholdCondition(item->conditions); if (monCfg) driver_temp_cfg_ = *monCfg; }
    item = getFaultCfg(MOTOR_OVER_TEMP);
    if (item) { report_motor_over_temp_ = true; auto monCfg = parseThresholdCondition(item->conditions); if (monCfg) motor_temp_cfg_ = *monCfg; }

    item = getFaultCfg(JNT_DRV_OVER_VOLT);
    report_joint_driver_over_volt_ = item.has_value();
    if (item) joint_drv_over_volt_severity_ = static_cast<uint8_t>(item->severity);
    item = getFaultCfg(JNT_DRV_UNDER_VOLT);
    report_joint_driver_under_volt_ = item.has_value();
    if (item) joint_drv_under_volt_severity_ = static_cast<uint8_t>(item->severity);
    item = getFaultCfg(JNT_DRV_OVER_CURR);
    report_joint_driver_over_curr_ = item.has_value();
    if (item) joint_drv_over_curr_severity_ = static_cast<uint8_t>(item->severity);
    item = getFaultCfg(JNT_DRV_TIMEOUT);
    report_joint_driver_timeout_ = item.has_value();
    if (item) joint_drv_timeout_severity_ = static_cast<uint8_t>(item->severity);
    item = getFaultCfg(TEMP_SENSOR_DISCON);
    report_temp_sensor_discon_ = item.has_value();
    if (item) temp_sensor_discon_severity_ = static_cast<uint8_t>(item->severity);
    item = getFaultCfg(ENC_CALIB_FAIL);
    report_enc_calib_fail_ = item.has_value();
    if (item) enc_calib_fail_severity_ = static_cast<uint8_t>(item->severity);
    item = getFaultCfg(JNT_ENC_NO_VAL);
    report_joint_enc_no_val_ = item.has_value();
    if (item) joint_enc_no_val_severity_ = static_cast<uint8_t>(item->severity);
    item = getFaultCfg(JNT_DRV_STATUS_ERR);
    report_joint_driver_status_error_ = item.has_value();
    if (item) joint_drv_status_err_severity_ = static_cast<uint8_t>(item->severity);
    item = getFaultCfg(JNT_VEL_OVER_LIM);
    vel_over_limit_cfg_.loadFrom(item);
    if (!vel_over_limit_cfg_.enabled) { vel_over_limit_cfg_.severity = Severity::WARN; }
    item = getFaultCfg(JNT_DATA_UPD_ERR);
    report_joint_data_update_error_ = item.has_value();
    if (item) {
        ApplyFaultTopSeverity(item, joint_data_upd_fault_severity_);
        for (const auto& cond : item->conditions) {
            if (cond.strategy == "thresholds") {
                for (const auto& cv : cond.values) {
                    if (cv.severity == Severity::ERROR && cv.value > 0) {
                        joint_update_timeout_threshold_ = cv.value;
                        joint_data_same_cnt_threshold_ = static_cast<int>(cv.value);
                        break;
                    }
                }
                break;
            }
        }
    }
    item = getFaultCfg(JNT_DATA_NAN); joint_nan_cfg_.loadFrom(item);
    item = getFaultCfg(JNT_POS_OVER_LIM); pos_over_limit_cfg_.loadFrom(item);
    item = getFaultCfg(BODY_POSE_ERR);
    report_body_pose_err_ = item.has_value();
    if (item) { ApplyFaultTopSeverity(item, body_pose_err_fault_severity_); }
    item = getFaultCfg(IMU_TIMEOUT);
    if (item) {
        report_imu_data_timeout_ = true;
        ApplyFaultTopSeverity(item, imu_timeout_fault_severity_);
        for (const auto& cond : item->conditions) {
            if (cond.strategy == "thresholds") {
                for (const auto& cv : cond.values) {
                    if (cv.severity == Severity::ERROR && cv.value > 0) { imu_update_timeout_threshold_ = cv.value; break; }
                }
                break;
            }
        }
    }
    item = getFaultCfg(IMU_NAN);
    report_imu_data_nan_ = item.has_value();
    if (item) { ApplyFaultTopSeverity(item, imu_nan_fault_severity_); }

    bit_JNT_DRV_OVER_VOLT_ = getBitFromFault(JNT_DRV_OVER_VOLT, bit_JNT_DRV_OVER_VOLT_);
    bit_JNT_DRV_UNDER_VOLT_ = getBitFromFault(JNT_DRV_UNDER_VOLT, bit_JNT_DRV_UNDER_VOLT_);
    bit_JNT_DRV_OVER_CURR_ = getBitFromFault(JNT_DRV_OVER_CURR, bit_JNT_DRV_OVER_CURR_);
    bit_MOTOR_OVER_TEMP_ = getBitFromFault(MOTOR_OVER_TEMP, bit_MOTOR_OVER_TEMP_);
    bit_JNT_DRV_OVER_TEMP_ = getBitFromFault(JNT_DRV_OVER_TEMP, bit_JNT_DRV_OVER_TEMP_);
    bit_JNT_DRV_TIMEOUT_ = getBitFromFault(JNT_DRV_TIMEOUT, bit_JNT_DRV_TIMEOUT_);
    bit_TEMP_SENSOR_DISCON_ = getBitFromFault(TEMP_SENSOR_DISCON, bit_TEMP_SENSOR_DISCON_);
    bit_enc_calib_fail_ = getBitFromFault(ENC_CALIB_FAIL, bit_enc_calib_fail_);
    bit_JNT_ENC_NO_VAL_ = getBitFromFault(JNT_ENC_NO_VAL, bit_JNT_ENC_NO_VAL_);
    bit_JNT_DRV_STATUS_ERR_ = getBitFromFault(JNT_DRV_STATUS_ERR, bit_JNT_DRV_STATUS_ERR_);

    // ── 填充 JointMonitorConfig ──
    joint_monitor_cfg_.pos_enabled          = pos_over_limit_cfg_.enabled;
    joint_monitor_cfg_.pos_count_threshold  = pos_over_limit_cfg_.count_threshold;
    joint_monitor_cfg_.pos_severity         = pos_over_limit_cfg_.severity;
    joint_monitor_cfg_.vel_enabled          = vel_over_limit_cfg_.enabled;
    joint_monitor_cfg_.vel_count_threshold  = vel_over_limit_cfg_.count_threshold;
    joint_monitor_cfg_.vel_severity         = vel_over_limit_cfg_.severity;
    joint_monitor_cfg_.nan_enabled          = joint_nan_cfg_.enabled;
    joint_monitor_cfg_.nan_count_threshold  = joint_nan_cfg_.count_threshold;
    joint_monitor_cfg_.nan_severity         = joint_nan_cfg_.severity;
    joint_monitor_cfg_.data_same_cnt_threshold = joint_data_same_cnt_threshold_;
    joint_monitor_cfg_.update_timeout_ms       = joint_update_timeout_threshold_;
    joint_monitor_cfg_.data_upd_severity       = joint_data_upd_fault_severity_;
    joint_monitor_cfg_.bit_drv_over_volt      = bit_JNT_DRV_OVER_VOLT_;
    joint_monitor_cfg_.bit_drv_under_volt     = bit_JNT_DRV_UNDER_VOLT_;
    joint_monitor_cfg_.bit_drv_over_curr      = bit_JNT_DRV_OVER_CURR_;
    joint_monitor_cfg_.bit_motor_over_temp    = bit_MOTOR_OVER_TEMP_;
    joint_monitor_cfg_.bit_drv_over_temp      = bit_JNT_DRV_OVER_TEMP_;
    joint_monitor_cfg_.bit_drv_timeout        = bit_JNT_DRV_TIMEOUT_;
    joint_monitor_cfg_.bit_temp_sensor_discon = bit_TEMP_SENSOR_DISCON_;
    joint_monitor_cfg_.bit_enc_calib_fail     = bit_enc_calib_fail_;
    joint_monitor_cfg_.bit_enc_no_val         = bit_JNT_ENC_NO_VAL_;
    joint_monitor_cfg_.bit_drv_status_err     = bit_JNT_DRV_STATUS_ERR_;

    safe_logger_.info("=== SafeController FaultConfig Loaded ===");
    safe_logger_.info("MOTOR_OVER_TEMP enabled=%d (LevelMonitor)", report_motor_over_temp_);
    safe_logger_.info("JNT_DRV_OVER_TEMP enabled=%d (LevelMonitor)", report_joint_driver_over_temp_);
    safe_logger_.info("JNT_DRV_OVER_VOLT enabled=%d bit=%d sev=%d", report_joint_driver_over_volt_, bit_JNT_DRV_OVER_VOLT_, joint_drv_over_volt_severity_);
    safe_logger_.info("JNT_DRV_UNDER_VOLT enabled=%d bit=%d sev=%d", report_joint_driver_under_volt_, bit_JNT_DRV_UNDER_VOLT_, joint_drv_under_volt_severity_);
    safe_logger_.info("JNT_DRV_OVER_CURR enabled=%d bit=%d sev=%d", report_joint_driver_over_curr_, bit_JNT_DRV_OVER_CURR_, joint_drv_over_curr_severity_);
    safe_logger_.info("JNT_DRV_TIMEOUT enabled=%d bit=%d sev=%d", report_joint_driver_timeout_, bit_JNT_DRV_TIMEOUT_, joint_drv_timeout_severity_);
    safe_logger_.info("TEMP_SENSOR_DISCON enabled=%d bit=%d sev=%d", report_temp_sensor_discon_, bit_TEMP_SENSOR_DISCON_, temp_sensor_discon_severity_);
    safe_logger_.info("ENC_CALIB_FAIL enabled=%d bit=%d sev=%d", report_enc_calib_fail_, bit_enc_calib_fail_, enc_calib_fail_severity_);
    safe_logger_.info("JNT_ENC_NO_VAL enabled=%d bit=%d sev=%d", report_joint_enc_no_val_, bit_JNT_ENC_NO_VAL_, joint_enc_no_val_severity_);
    safe_logger_.info("JNT_DRV_STATUS_ERR enabled=%d bit=%d sev=%d", report_joint_driver_status_error_, bit_JNT_DRV_STATUS_ERR_, joint_drv_status_err_severity_);
    safe_logger_.info("JNT_VEL_OVER_LIM enabled=%d debounce=%d sev=%d", vel_over_limit_cfg_.enabled, vel_over_limit_cfg_.count_threshold, vel_over_limit_cfg_.severity);
    safe_logger_.info("JNT_DATA_UPD_ERR enabled=%d timeout=%.1f same_cnt=%d sev=%d", report_joint_data_update_error_, joint_update_timeout_threshold_, joint_data_same_cnt_threshold_, joint_data_upd_fault_severity_);
    safe_logger_.info("JNT_DATA_NAN enabled=%d debounce=%d sev=%d", joint_nan_cfg_.enabled, joint_nan_cfg_.count_threshold, joint_nan_cfg_.severity);
    safe_logger_.info("JNT_POS_OVER_LIM enabled=%d debounce=%d sev=%d", pos_over_limit_cfg_.enabled, pos_over_limit_cfg_.count_threshold, pos_over_limit_cfg_.severity);
    safe_logger_.info("BODY_POSE_ERR enabled=%d sev=%d", report_body_pose_err_, body_pose_err_fault_severity_);
    safe_logger_.info("IMU_TIMEOUT enabled=%d timeout=%.1f sev=%d", report_imu_data_timeout_, imu_update_timeout_threshold_, imu_timeout_fault_severity_);
    safe_logger_.info("IMU_NAN enabled=%d sev=%d", report_imu_data_nan_, imu_nan_fault_severity_);

    // ── 主板电压阈值 + 严重等级（从故障配置 [rule] 读取，支持 custom.CR1 覆盖）──
    //    输入：MB_INPUT_UNDER_VOLT(0x8313) / MB_INPUT_OVER_VOLT(0x8312)
    //    输出：MB_OUTPUT_UNDER_VOLT(0x8315) / MB_OUTPUT_OVER_VOLT(0x8314)
    //    输入/输出共用同一套阈值，分别取 WARN 级别值
    {
        // MB_INPUT_UNDER_VOLT: 读取阈值(under) + severity
        auto item = getFaultCfg(MB_INPUT_UNDER_VOLT);
        report_mainboard_input_under_volt_ = item.has_value();
        if (item) {
            voltage_input_under_severity_ = static_cast<uint8_t>(item->severity);
            auto monCfg = parseThresholdCondition(item->conditions);
            if (monCfg) {
                for (const auto& [sev, val] : monCfg->thresholds) {
                    if (sev == Severity::WARN) { voltage_under_threshold_ = static_cast<float>(val); break; }
                }
            }
        }
        // MB_INPUT_OVER_VOLT: 读取阈值(over) + severity（独立变量，不与 under 共用）
        item = getFaultCfg(MB_INPUT_OVER_VOLT);
        report_mainboard_input_over_volt_ = item.has_value();
        if (item) {
            voltage_input_over_severity_ = static_cast<uint8_t>(item->severity);
            auto monCfg = parseThresholdCondition(item->conditions);
            if (monCfg) {
                for (const auto& [sev, val] : monCfg->thresholds) {
                    if (sev == Severity::WARN) { voltage_over_threshold_ = static_cast<float>(val); break; }
                }
            }
        }
        // MB_OUTPUT_UNDER_VOLT / MB_OUTPUT_OVER_VOLT: 分别读取 severity
        item = getFaultCfg(MB_OUTPUT_UNDER_VOLT);
        report_mainboard_output_under_volt_ = item.has_value();
        if (item) { voltage_output_under_severity_ = static_cast<uint8_t>(item->severity); }
        item = getFaultCfg(MB_OUTPUT_OVER_VOLT);
        report_mainboard_output_over_volt_ = item.has_value();
        if (item) { voltage_output_over_severity_ = static_cast<uint8_t>(item->severity); }
    }
    // 合理性校验
    if (voltage_under_threshold_ <= 0.0f) {
        safe_logger_.warn("Invalid voltage_under_threshold=%.1f from fault config, reset to 54.0", voltage_under_threshold_);
        voltage_under_threshold_ = 54.0f;
    }
    if (voltage_over_threshold_ <= voltage_under_threshold_) {
        safe_logger_.warn("Invalid voltage_over_threshold=%.1f (<= under=%.1f) from fault config, reset to 90.0",
                          voltage_over_threshold_, voltage_under_threshold_);
        voltage_over_threshold_ = 90.0f;
    }

    // ── 电机通信状态故障严重等级 + 阈值（从故障配置 [rule] 读取，支持 custom 覆盖）──
    {
        auto item = getFaultCfg(JNT_DRV_DISABLED);
        report_motor_comm_disabled_ = item.has_value();
        if (item) { motor_comm_disabled_severity_ = static_cast<uint8_t>(item->severity); }
        item = getFaultCfg(JNT_ERR_FRAME_EXCESS);
        report_motor_comm_err_ = item.has_value();
        if (item) {
            motor_comm_err_severity_ = static_cast<uint8_t>(item->severity);
            auto monCfg = parseThresholdCondition(item->conditions);
            if (monCfg) {
                for (const auto& [sev, val] : monCfg->thresholds) {
                    if (sev == Severity::WARN) { motor_comm_err_threshold_ = static_cast<int>(val); break; }
                }
            }
        }
        item = getFaultCfg(JNT_LOST_FRAME_EXCESS);
        report_motor_comm_loss_ = item.has_value();
        if (item) {
            motor_comm_loss_severity_ = static_cast<uint8_t>(item->severity);
            auto monCfg = parseThresholdCondition(item->conditions);
            if (monCfg) {
                for (const auto& [sev, val] : monCfg->thresholds) {
                    if (sev == Severity::WARN) { motor_comm_loss_threshold_ = static_cast<int>(val); break; }
                }
            }
        }
        item = getFaultCfg(JNT_BUS_OFF);
        report_motor_comm_busoff_ = item.has_value();
        if (item) { motor_comm_busoff_severity_ = static_cast<uint8_t>(item->severity); }
    }

    safe_logger_.info("VOLTAGE_LIMIT under=%.1f over=%.1f in_under_sev=%d(rep=%d) in_over_sev=%d(rep=%d) out_under_sev=%d(rep=%d) out_over_sev=%d(rep=%d) (from fault config)",
                      voltage_under_threshold_, voltage_over_threshold_,
                      voltage_input_under_severity_, report_mainboard_input_under_volt_,
                      voltage_input_over_severity_, report_mainboard_input_over_volt_,
                      voltage_output_under_severity_, report_mainboard_output_under_volt_,
                      voltage_output_over_severity_, report_mainboard_output_over_volt_);
    safe_logger_.info("MOTOR_COMM sev: disabled=%d(rep=%d) err=%d(rep=%d,thr=%d) loss=%d(rep=%d,thr=%d) busoff=%d(rep=%d) (from fault config)",
                      motor_comm_disabled_severity_, report_motor_comm_disabled_,
                      motor_comm_err_severity_, report_motor_comm_err_, motor_comm_err_threshold_,
                      motor_comm_loss_severity_, report_motor_comm_loss_, motor_comm_loss_threshold_,
                      motor_comm_busoff_severity_, report_motor_comm_busoff_);
    safe_logger_.info("=== End FaultConfig ===");
}

void SafeController::LoadPostureLimitConfig(){
    std::string toml_path;
    for (const char* candidate : {"/var/opt/robot/conf/motion_master/config.toml", "conf/config.toml"}) {
        if (std::filesystem::exists(candidate)) { toml_path = candidate; break; }
    }
    if (!toml_path.empty()) {
        try {
            auto toml_cfg = toml::parse(toml_path);
            if (toml_cfg.count("safe_controller")) {
                const auto& sc = toml::find(toml_cfg, "safe_controller");
                report_only_ = toml::find_or<bool>(sc, "report_only", false);
            }
            const auto& pl = toml::find(toml_cfg, "posture_limit");
            posture_limit_enable_ = toml::find_or<bool>(pl, "enable", true);
            roll_threshold_deg_ = static_cast<float>(toml::find_or<double>(pl, "roll_threshold_deg", 15.0));
            pitch_threshold_deg_ = static_cast<float>(toml::find_or<double>(pl, "pitch_threshold_deg", 20.0));
            hysteresis_ms_ = toml::find_or<int>(pl, "hysteresis_ms", 100);
            if (roll_threshold_deg_ <= 0.0f) { safe_logger_.warn("Invalid roll_threshold_deg=%.1f, reset to 15.0", roll_threshold_deg_); roll_threshold_deg_ = 15.0f; }
            if (pitch_threshold_deg_ <= 0.0f) { safe_logger_.warn("Invalid pitch_threshold_deg=%.1f, reset to 20.0", pitch_threshold_deg_); pitch_threshold_deg_ = 20.0f; }
            if (hysteresis_ms_ <= 0) { safe_logger_.warn("Invalid hysteresis_ms=%d, reset to 100", hysteresis_ms_); hysteresis_ms_ = 100; }
        } catch (const std::exception& e) {
            safe_logger_.warn("Failed to parse [posture_limit] config: %s, using defaults", e.what());
        }
    } else {
        safe_logger_.warn("config.toml not found, posture_limit using defaults");
    }
    posture_exceed_threshold_ = hysteresis_ms_;
    auto& rp = RuntimeParams::Instance();
    rp.SetBool  ("safe_controller", "report_only",         report_only_);
    rp.SetBool  ("posture_limit",   "enable",              posture_limit_enable_);
    rp.SetDouble("posture_limit",   "roll_threshold_deg",  static_cast<double>(roll_threshold_deg_));
    rp.SetDouble("posture_limit",   "pitch_threshold_deg", static_cast<double>(pitch_threshold_deg_));
    rp.SetInt   ("posture_limit",   "hysteresis_ms",       static_cast<long long>(hysteresis_ms_));
    safe_logger_.info("POSTURE_LIMIT enabled=%d roll=%.1f pitch=%.1f hysteresis=%d threshold=%d",
                     posture_limit_enable_, roll_threshold_deg_, pitch_threshold_deg_, hysteresis_ms_, posture_exceed_threshold_);
    safe_logger_.info("REPORT_ONLY mode=%d (true: only publish faults, do NOT trigger soft e-stop)", report_only_);
}
