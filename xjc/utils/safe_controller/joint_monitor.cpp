#include "joint_monitor.hpp"
#include <cmath>

void JointMonitor::init(int dof_num, const JointMonitorConfig& cfg) {
    dof_num_ = dof_num;
    cfg_     = cfg;
    data_same_cnt_.assign(dof_num, 0);
    posoutlimit_cnt_.assign(dof_num, 0);
    veloutlimit_cnt_.assign(dof_num, 0);
    pos_nan_cnt_.assign(dof_num, 0);
    vel_nan_cnt_.assign(dof_num, 0);
    last_pos_.resize(dof_num);
    last_vel_.resize(dof_num);
    last_tau_.resize(dof_num);
    driver_status_prev_.assign(dof_num, 0);
    first_check_done_ = false;
}

static bool g_first_flag = false;  // 复现原 static 局部变量行为

uint32_t JointMonitor::checkDataUpdate(
    double current_time_ms, double ri_ts,
    const types::VecXf& pos, const types::VecXf& vel, const types::VecXf& tau,
    std::vector<uint8_t>& out_severity, bool& out_not_updated)
{
    out_severity.clear();
    if (!first_check_done_) {
        first_check_done_ = true;
        last_ri_ts_ = ri_ts;
        last_pos_   = pos;
        last_vel_   = vel;
        last_tau_   = tau;
    }

    bool res = true;
    if (last_ri_ts_ != ri_ts) {
        last_ri_ts_    = ri_ts;
        ri_check_time_ = current_time_ms;
        g_first_flag   = true;
    }

    uint32_t mask = 0;
    for (int i = 0; i < dof_num_; ++i) {
        if (pos(i) == last_pos_(i) && vel(i) == last_vel_(i) && tau(i) == last_tau_(i)) {
            data_same_cnt_[i]++;
        } else {
            data_same_cnt_[i] = 0;
        }
        if (data_same_cnt_[i] > cfg_.data_same_cnt_threshold) {
            mask |= (1u << i);
            out_severity.push_back(cfg_.data_upd_severity);
            res = false;
        }
    }

    last_pos_ = pos;
    last_vel_ = vel;
    last_tau_ = tau;
    out_not_updated = !res;
    return mask;
}

uint32_t JointMonitor::checkNan(
    const types::VecXf& pos, const types::VecXf& vel, const types::VecXf& tau,
    std::vector<uint8_t>& out_severity)
{
    out_severity.clear();
    uint32_t mask = 0;
    for (int i = 0; i < dof_num_; ++i) {
        if (std::isnan(pos(i)) || std::isnan(vel(i)) || std::isnan(tau(i))) {
            pos_nan_cnt_[i]++;
        } else {
            pos_nan_cnt_[i] = 0;
        }
        if (pos_nan_cnt_[i] > cfg_.nan_count_threshold) {
            mask |= (1u << i);
            out_severity.push_back(cfg_.nan_severity);
        }
    }
    return mask;
}

uint32_t JointMonitor::checkPosLimit(
    const types::VecXf& pos,
    const types::VecXf& pos_lower, const types::VecXf& pos_upper,
    std::vector<uint8_t>& out_severity)
{
    out_severity.clear();
    uint32_t mask = 0;
    for (int i = 0; i < dof_num_; ++i) {
        if (std::isnan(pos_lower(i)) || std::isnan(pos_upper(i)) || pos(i) > pos_upper(i) + 0.1f || pos(i) < pos_lower(i) - 0.1f) {
            posoutlimit_cnt_[i]++;
        } else {
            posoutlimit_cnt_[i] = 0;
        }
        if (posoutlimit_cnt_[i] > cfg_.pos_count_threshold) {
            mask |= (1u << i);
            out_severity.push_back(cfg_.pos_severity);
        }
    }
    return mask;
}

uint32_t JointMonitor::checkVelLimit(
    const types::VecXf& vel, const types::VecXf& vel_limit,
    std::vector<uint8_t>& out_severity)
{
    out_severity.clear();
    uint32_t mask = 0;
    for (int i = 0; i < dof_num_; ++i) {
        if (std::isnan(vel(i)) || std::abs(vel(i)) > vel_limit(i) + 0.1f) {
            veloutlimit_cnt_[i]++;
        } else {
            veloutlimit_cnt_[i] = 0;
        }
        if (veloutlimit_cnt_[i] > cfg_.vel_count_threshold) {
            mask |= (1u << i);
            out_severity.push_back(cfg_.vel_severity);
        }
    }
    return mask;
}

uint32_t JointMonitor::checkDriverStatus(
    const std::vector<uint16_t>& status_word,
    uint32_t& out_over_volt, uint32_t& out_under_volt,
    uint32_t& out_over_curr, uint32_t& out_motor_temp,
    uint32_t& out_drv_temp, uint32_t& out_timeout,
    uint32_t& out_temp_sensor, uint32_t& out_enc_fail,
    uint32_t& out_enc_offline, uint32_t& out_drv_discon,
    std::map<int, uint16_t>& out_error_map)
{
    uint32_t mask = 0;
    out_over_volt  = 0; out_under_volt = 0; out_over_curr = 0;
    out_motor_temp = 0; out_drv_temp   = 0; out_timeout   = 0;
    out_temp_sensor = 0; out_enc_fail  = 0; out_enc_offline = 0;
    out_drv_discon  = 0;

    for (size_t i = 0; i < status_word.size(); ++i) {
        if (status_word[i] != 1) {
            out_error_map[static_cast<uint8_t>(i)] = status_word[i];
            mask |= (1u << i);
            if (status_word[i] == 0) {
                out_timeout |= (1u << i);
            } else {
                uint16_t bit_state = static_cast<uint16_t>(status_word[i] - 1);
                if (bit_state & (1u << cfg_.bit_drv_over_volt))  out_over_volt  |= (1u << i);
                if (bit_state & (1u << cfg_.bit_drv_under_volt)) out_under_volt |= (1u << i);
                if (bit_state & (1u << cfg_.bit_drv_over_curr))  out_over_curr  |= (1u << i);
                if (bit_state & (1u << cfg_.bit_motor_over_temp)) out_motor_temp |= (1u << i);
                if (bit_state & (1u << cfg_.bit_drv_over_temp))  out_drv_temp   |= (1u << i);
                if (bit_state & (1u << cfg_.bit_drv_timeout))    out_timeout    |= (1u << i);
                if (bit_state & (1u << cfg_.bit_temp_sensor_discon)) out_temp_sensor |= (1u << i);
                if (bit_state & (1u << cfg_.bit_enc_calib_fail)) out_enc_fail     |= (1u << i);
                if (bit_state & (1u << cfg_.bit_enc_no_val))     out_enc_offline  |= (1u << i);
                if (bit_state & (1u << cfg_.bit_drv_status_err)) out_drv_discon   |= (1u << i);
            }
        }
        // 记录状态变化（原 safe_logger_.warn 逻辑由调用方处理，这里只记录）
        if (i < driver_status_prev_.size()) {
            driver_status_prev_[i] = static_cast<uint16_t>(status_word[i] - 1);
        }
    }
    return mask;
}
