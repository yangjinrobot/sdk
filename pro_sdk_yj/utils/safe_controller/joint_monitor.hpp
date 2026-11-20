#pragma once
#include "common_types.h"
#include <vector>
#include <cstdint>
#include <map>

// ── 配置（由 LoadFaultConfig 填充）───────────────────────────────────────────
struct JointMonitorConfig {
    // Debounced fault configs (flattened to avoid circular dependency)
    bool    pos_enabled = false;   int pos_count_threshold = 30;  uint8_t pos_severity = 3;
    bool    vel_enabled = false;   int vel_count_threshold = 30;  uint8_t vel_severity = 3;
    bool    nan_enabled = false;   int nan_count_threshold = 30;  uint8_t nan_severity = 3;

    // Joint data update
    int     data_same_cnt_threshold = 50;
    double  update_timeout_ms       = 30;
    uint8_t data_upd_severity       = 3;  // Severity::FATAL

    // 驱动器状态位（yaml bit_match）
    int bit_drv_over_volt      = 0;
    int bit_drv_under_volt     = 1;
    int bit_drv_over_curr      = 2;
    int bit_motor_over_temp    = 3;
    int bit_drv_over_temp      = 4;
    int bit_drv_timeout        = 5;
    int bit_temp_sensor_discon = 6;
    int bit_enc_calib_fail     = 7;
    int bit_enc_no_val         = 8;
    int bit_drv_status_err     = 9;
};

// ── 关节故障 + 驱动器状态检测器 ─────────────────────────────────────────────
class JointMonitor {
public:
    void init(int dof_num, const JointMonitorConfig& cfg);

    // ── 每周期逐项调用（保持原名 + 相同调用模式）──

    // 关节数据更新检测 → update_mask（调用方写入 atomic + reportFault）
    uint32_t checkDataUpdate(double current_time_ms, double ri_ts,
                             const types::VecXf& pos, const types::VecXf& vel,
                             const types::VecXf& tau,
                             std::vector<uint8_t>& out_severity,
                             bool& out_not_updated);

    // 关节 NaN 检测 → nan_mask
    uint32_t checkNan(const types::VecXf& pos, const types::VecXf& vel,
                      const types::VecXf& tau,
                      std::vector<uint8_t>& out_severity);

    // 关节位置超限 → pos_error_mask
    uint32_t checkPosLimit(const types::VecXf& pos,
                           const types::VecXf& pos_lower,
                           const types::VecXf& pos_upper,
                           std::vector<uint8_t>& out_severity);

    // 关节速度超限 → vel_error_mask
    uint32_t checkVelLimit(const types::VecXf& vel,
                           const types::VecXf& vel_limit,
                           std::vector<uint8_t>& out_severity);

    // 驱动器状态 → driver_status_mask + 10 个子故障掩码
    uint32_t checkDriverStatus(const std::vector<uint16_t>& status_word,
                               uint32_t& out_over_volt, uint32_t& out_under_volt,
                               uint32_t& out_over_curr, uint32_t& out_motor_temp,
                               uint32_t& out_drv_temp, uint32_t& out_timeout,
                               uint32_t& out_temp_sensor, uint32_t& out_enc_fail,
                               uint32_t& out_enc_offline, uint32_t& out_drv_discon,
                               std::map<int, uint16_t>& out_error_map);

private:
    std::vector<int> data_same_cnt_, posoutlimit_cnt_, veloutlimit_cnt_;
    std::vector<int> pos_nan_cnt_, vel_nan_cnt_;
    types::VecXf last_pos_, last_vel_, last_tau_;
    double ri_check_time_ = 0, last_ri_ts_ = 0;
    bool   first_check_done_ = false;

    std::vector<uint16_t> driver_status_prev_;
    JointMonitorConfig cfg_;
    int dof_num_ = 0;
};
