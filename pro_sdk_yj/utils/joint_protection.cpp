/**
 * @file joint_protection.cpp
 * @brief SetJointCommand 三级保护检查器实现
 * @author deeprobotics
 * @version 1.0
 * @date 2026-04-08
 *
 * @copyright Copyright (c) 2026  DeepRobotics
 *
 */

#include "joint_protection.hpp"
#include "parameters/control_parameters.h"
#include "runtime_params.hpp"

#include <cmath>
#include <algorithm>

// ============================================================================
// JointProtectionState
// ============================================================================

void JointProtectionState::reset(int dof_num) {
    l2_counters.assign(dof_num, {0, 0, 0, 0, 0});
    l1_triggered = false;
    l1_joint_idx = -1;
    l1_dim_idx = -1;
    l1_trigger_count = 0;
    l2_clamp_count = 0;
    l3_log_count = 0;
}

// ============================================================================
// JointProtectionChecker — 辅助枚举
// ============================================================================

namespace {

enum class JointGroup {
    LEG,
    ARM,
    WAIST,
    NECK
};

/**
 * @brief 判断关节索引所属关节组
 * @param idx 关节索引
 * @param dof 总自由度数
 * @param cp  控制参数指针
 */
JointGroup jointGroupOf(int idx, int dof, const ControlParameters* cp) {
    if (dof == 31) {
        // 腰[0-2], 左臂[3-9], 右臂[10-16], 左腿[17-22], 右腿[23-28], 头[29-30]
        if (idx < 3) return JointGroup::WAIST;
        if (idx < 10) return JointGroup::ARM;
        if (idx < 17) return JointGroup::ARM;
        if (idx < 23) return JointGroup::LEG;
        if (idx < 29) return JointGroup::LEG;
        return JointGroup::NECK;
    } else if (dof == 21) {
        // 腰[0], 左臂[1-4], 右臂[5-8], 左腿[9-14], 右腿[15-20]
        if (idx < cp->waist_dof_num_) return JointGroup::WAIST;
        int arm_end = cp->waist_dof_num_ + 2 * cp->arm_dof_num_;
        if (idx < arm_end) return JointGroup::ARM;
        return JointGroup::LEG;
    } else if (dof == 12) {
        // 全部为腿
        return JointGroup::LEG;
    } else if (dof == 14) {
        // 全部为臂
        return JointGroup::ARM;
    }
    // 默认归为腿
    return JointGroup::LEG;
}

/**
 * @brief 从关节索引推导该关节在其所属组内的局部索引
 * @param idx  全局关节索引
 * @param dof  总自由度数
 * @param cp   控制参数指针
 * @return 组内局部索引（用于从 kp_/kd_ 向量中取值）
 */
int jointLocalIndex(int idx, int dof, const ControlParameters* cp) {
    if (dof == 31) {
        // 腰[0-2], 左臂[3-9], 右臂[10-16], 左腿[17-22], 右腿[23-28], 头[29-30]
        if (idx < 3)  return idx;             // waist: 局部索引 0~2
        if (idx < 10) return idx - 3;         // 左臂:  局部索引 0~6
        if (idx < 17) return idx - 10;        // 右臂:  局部索引 0~6
        if (idx < 23) return idx - 17;        // 左腿:  局部索引 0~5
        if (idx < 29) return idx - 23;        // 右腿:  局部索引 0~5
        return idx - 29;                      // 头:    局部索引 0~1
    } else if (dof == 21) {
        // 腰[0], 左臂[1-waist+arm], 右臂, 左腿, 右腿
        int w = cp->waist_dof_num_;
        int a = cp->arm_dof_num_;
        int l = cp->leg_dof_num_;
        if (idx < w)             return idx;              // waist
        if (idx < w + a)         return idx - w;          // 左臂
        if (idx < w + 2*a)       return idx - w - a;      // 右臂
        if (idx < w + 2*a + l)   return idx - w - 2*a;   // 左腿
        return idx - w - 2*a - l;                         // 右腿
    } else if (dof == 12) {
        // 左腿[0-5], 右腿[6-11]
        return idx % 6;
    } else if (dof == 14) {
        // 左臂[0-6], 右臂[7-13]
        return idx % 7;
    }
    return 0;
}

const char* groupToString(JointGroup grp) {
    switch (grp) {
        case JointGroup::LEG:   return "leg";
        case JointGroup::ARM:   return "arm";
        case JointGroup::WAIST: return "waist";
        case JointGroup::NECK:  return "neck";
    }
    return "unknown";
}

}  // namespace

// ============================================================================
// JointProtectionChecker — 公共接口
// ============================================================================

bool JointProtectionChecker::isEnabled() const {
    return enabled_;
}

void JointProtectionChecker::init(int dof_num, const ControlParameters* cp_ptr, const toml::value& config, FaultClient* fault_client) {
    dof_num_ = dof_num;

    // 从 TOML [joint_protection] 读取全局参数
    try {
        const auto& jp = toml::find(config, "joint_protection");
        enabled_        = toml::find_or<bool>(jp, "enabled", false);
        l2_threshold_   = toml::find_or<int>(jp, "l2_threshold", 10);
        l2_decay_rate_  = toml::find_or<int>(jp, "l2_decay_rate", 1);
        damping_kd_     = static_cast<float>(toml::find_or<double>(jp, "damping_kd", 10.0));
        margin_l1_      = static_cast<float>(toml::find_or<double>(jp, "margin_l1", 0.5));
        l3_log_interval_ = toml::find_or<int>(jp, "l3_log_interval", 100);
        l2_log_interval_ = toml::find_or<int>(jp, "l2_log_interval", 100);
        enable_fault_report_  = toml::find_or<bool>(jp, "enable_fault_report", true);
        enable_damping_mode_  = toml::find_or<bool>(jp, "enable_damping_mode", true);
    } catch (const std::exception& e) {
        logger_.warn("Failed to parse [joint_protection] from TOML: %s, protection disabled", e.what());
        enabled_ = false;
        return;
    }

    // 保障日志间隔为正数，避免 %0 崩溃
    if (l2_log_interval_ <= 0) {
        logger_.warn("Invalid l2_log_interval=%d, reset to 100", l2_log_interval_);
        l2_log_interval_ = 100;
    }
    if (l3_log_interval_ <= 0) {
        logger_.warn("Invalid l3_log_interval=%d, reset to 100", l3_log_interval_);
        l3_log_interval_ = 100;
    }
    // 保障业务参数合法性
    if (l2_threshold_ <= 0) {
        logger_.warn("Invalid l2_threshold=%d, reset to 10", l2_threshold_);
        l2_threshold_ = 10;
    }
    if (l2_decay_rate_ < 0) {
        logger_.warn("Invalid l2_decay_rate=%d, reset to 1", l2_decay_rate_);
        l2_decay_rate_ = 1;
    }
    if (damping_kd_ <= 0.0f) {
        logger_.warn("Invalid damping_kd=%.2f, reset to 10.0", damping_kd_);
        damping_kd_ = 10.0f;
    }
    if (margin_l1_ < 0.0f) {
        logger_.warn("Invalid margin_l1=%.2f, reset to 0.5", margin_l1_);
        margin_l1_ = 0.5f;
    }

    if (!enabled_) {
        logger_.info("JointProtection is disabled by config");
        return;
    }

    // 使用外部传入的故障上报客户端（由 SafeController 提供）
    fault_client_ = fault_client;
    last_protection_fault_code_ = 0;

    initLimitsFromConfig(cp_ptr, config);
    state_.reset(dof_num_);
    l3_frame_counter_ = 0;

    logger_.info("JointProtection initialized: dof=%d, l2_threshold=%d, margin_l1=%.2f, l2_log_interval=%d, l3_log_interval=%d",
                 dof_num_, l2_threshold_, margin_l1_, l2_log_interval_, l3_log_interval_);
    logger_.info("JointProtection switches: enable_fault_report=%s, enable_damping_mode=%s",
                 enable_fault_report_ ? "true" : "false",
                 enable_damping_mode_ ? "true" : "false");

    // 上报实际生效值至运行期参数快照
    {
        auto& rp = RuntimeParams::Instance();
        rp.SetBool  ("joint_protection", "enabled",             enabled_);
        rp.SetInt   ("joint_protection", "l2_threshold",        static_cast<long long>(l2_threshold_));
        rp.SetInt   ("joint_protection", "l2_decay_rate",       static_cast<long long>(l2_decay_rate_));
        rp.SetDouble("joint_protection", "damping_kd",          static_cast<double>(damping_kd_));
        rp.SetDouble("joint_protection", "margin_l1",           static_cast<double>(margin_l1_));
        rp.SetInt   ("joint_protection", "l2_log_interval",     static_cast<long long>(l2_log_interval_));
        rp.SetInt   ("joint_protection", "l3_log_interval",     static_cast<long long>(l3_log_interval_));
        rp.SetBool  ("joint_protection", "enable_fault_report", enable_fault_report_);
        rp.SetBool  ("joint_protection", "enable_damping_mode", enable_damping_mode_);
    }
}

// ============================================================================
// initLimitsFromConfig
// ============================================================================

void JointProtectionChecker::initLimitsFromConfig(const ControlParameters* cp_ptr, const toml::value& config) {
    limits_.resize(dof_num_);

    // 构建全关节的位置上下限向量 (复用 safe_controller 的拼接逻辑)
    VecXf joint_pos_lower = VecXf::Constant(dof_num_, -3.14f);
    VecXf joint_pos_upper = VecXf::Constant(dof_num_,  3.14f);

    if (dof_num_ == 31) {
        joint_pos_lower << cp_ptr->waist_joint_lower_, cp_ptr->arm_joint_lower_, cp_ptr->arm_joint_lower_,
                           cp_ptr->leg_joint_lower_, cp_ptr->leg_joint_lower_, cp_ptr->neck_joint_lower_;
        joint_pos_upper << cp_ptr->waist_joint_upper_, cp_ptr->arm_joint_upper_, cp_ptr->arm_joint_upper_,
                           cp_ptr->leg_joint_upper_, cp_ptr->leg_joint_upper_, cp_ptr->neck_joint_upper_;
        // 镜像关节修正 (与 safe_controller 一致)
        joint_pos_lower(11) = -cp_ptr->arm_joint_upper_(1);
        joint_pos_upper(11) = -cp_ptr->arm_joint_lower_(1);
        joint_pos_lower(12) = -cp_ptr->arm_joint_upper_(2);
        joint_pos_upper(12) = -cp_ptr->arm_joint_lower_(2);
        joint_pos_lower(24) = -cp_ptr->leg_joint_upper_(1);
        joint_pos_upper(24) = -cp_ptr->leg_joint_lower_(1);
        joint_pos_lower(25) = -cp_ptr->leg_joint_upper_(2);
        joint_pos_upper(25) = -cp_ptr->leg_joint_lower_(2);
    } else if (dof_num_ == 21) {
        joint_pos_lower << cp_ptr->waist_joint_lower_, cp_ptr->arm_joint_lower_, cp_ptr->arm_joint_lower_,
                           cp_ptr->leg_joint_lower_, cp_ptr->leg_joint_lower_;
        joint_pos_upper << cp_ptr->waist_joint_upper_, cp_ptr->arm_joint_upper_, cp_ptr->arm_joint_upper_,
                           cp_ptr->leg_joint_upper_, cp_ptr->leg_joint_upper_;
        joint_pos_lower(6)  = -cp_ptr->arm_joint_upper_(1);
        joint_pos_upper(6)  = -cp_ptr->arm_joint_lower_(1);
        joint_pos_lower(7)  = -cp_ptr->arm_joint_upper_(2);
        joint_pos_upper(7)  = -cp_ptr->arm_joint_lower_(2);
        joint_pos_lower(16) = -cp_ptr->leg_joint_upper_(1);
        joint_pos_upper(16) = -cp_ptr->leg_joint_lower_(1);
        joint_pos_lower(17) = -cp_ptr->leg_joint_upper_(2);
        joint_pos_upper(17) = -cp_ptr->leg_joint_lower_(2);
    } else {
        logger_.warn("Unknown dof_num=%d, using default pos limits ±3.14", dof_num_);
    }

    for (int i = 0; i < dof_num_; ++i) {
        auto& lim = limits_[i];
        JointGroup grp = jointGroupOf(i, dof_num_, cp_ptr);
        int local_idx = jointLocalIndex(i, dof_num_, cp_ptr);

        // --- 从 ControlParameters 获取该关节的 kp/kd 参考值 ---
        float ref_kp = 100.0f;  // 默认值
        float ref_kd = 10.0f;   // 默认值
        bool found = false;

        switch (grp) {
            case JointGroup::LEG:
                if (local_idx < static_cast<int>(cp_ptr->leg_kp_.size())) {
                    ref_kp = cp_ptr->leg_kp_(local_idx);
                    ref_kd = cp_ptr->leg_kd_(local_idx);
                    found = true;
                }
                break;
            case JointGroup::ARM:
                if (local_idx < static_cast<int>(cp_ptr->arm_kp_.size())) {
                    ref_kp = cp_ptr->arm_kp_(local_idx);
                    ref_kd = cp_ptr->arm_kd_(local_idx);
                    found = true;
                }
                break;
            case JointGroup::WAIST:
                if (local_idx < static_cast<int>(cp_ptr->waist_kp_.size())) {
                    ref_kp = cp_ptr->waist_kp_(local_idx);
                    ref_kd = cp_ptr->waist_kd_(local_idx);
                    found = true;
                }
                break;
            case JointGroup::NECK:
                if (local_idx < static_cast<int>(cp_ptr->neck_kp_.size())) {
                    ref_kp = cp_ptr->neck_kp_(local_idx);
                    ref_kd = cp_ptr->neck_kd_(local_idx);
                    found = true;
                }
                break;
        }

        if (!found) {
            logger_.warn("Joint[%d] group=%s local_idx=%d: kp/kd ref not found, using defaults kp=%.1f kd=%.1f",
                         i, groupToString(grp), local_idx, ref_kp, ref_kd);
        }

        // --- kp 三级阈值：L3 = ref×1.5, L2 = L3×2, L1 = L3×3 ---
        lim.kp_nominal_max = ref_kp * 1.5f;
        lim.kp_soft_max    = lim.kp_nominal_max * 5.0f;
        lim.kp_hard_max    = lim.kp_nominal_max * 10.0f;

        // --- kd 三级阈值：L3 = ref×1.5, L2 = L3×2, L1 = L3×3 ---
        lim.kd_nominal_max = ref_kd * 1.5f;
        lim.kd_soft_max    = lim.kd_nominal_max * 5.0f;
        lim.kd_hard_max    = lim.kd_nominal_max * 10.0f;

        // --- 位置限位 ---
        // L2 = ControlParameters 的限位值
        float pos_l = joint_pos_lower(i);
        float pos_u = joint_pos_upper(i);
        lim.pos_soft_lower = pos_l;
        lim.pos_soft_upper = pos_u;
        // L1 = L2 ± margin_l1
        lim.pos_hard_lower = pos_l - margin_l1_;
        lim.pos_hard_upper = pos_u + margin_l1_;
        // L3 = L2 范围的 99%（取中间 99% 区间）
        float center = (pos_l + pos_u) * 0.5f;
        float half_range = (pos_u - pos_l) * 0.5f;
        lim.pos_nominal_lower = center - half_range * 0.99f;
        lim.pos_nominal_upper = center + half_range * 0.99f;

        // --- 速度限位 ---
        float vel_limit = (i < cp_ptr->joint_vel_limit_.size()) ? cp_ptr->joint_vel_limit_(i) : 20.0f;
        lim.vel_soft_max    = vel_limit;
        lim.vel_hard_max    = vel_limit * 2.0f;
        lim.vel_nominal_max = vel_limit * 0.99f;

        // --- 力矩限位 ---
        float tor_limit = (i < cp_ptr->torque_limit_.size()) ? cp_ptr->torque_limit_(i) : 100.0f;
        lim.tor_soft_max    = tor_limit;
        lim.tor_hard_max    = tor_limit * 2.0f;
        lim.tor_nominal_max = tor_limit * 0.99f;

        logger_.info("Joint[%d] group=%s pos=[%.3f,%.3f] vel=%.1f tor=%.1f kp_soft=%.0f kd_soft=%.0f",
                     i, groupToString(grp), pos_l, pos_u, vel_limit, tor_limit,
                     lim.kp_soft_max, lim.kd_soft_max);
    }
}

// ============================================================================
// check — 核心保护逻辑
// ============================================================================

ProtectionResult JointProtectionChecker::check(Eigen::Matrix<float, Eigen::Dynamic, 5>& input) {
    if (!enabled_) return ProtectionResult::PASS;
    bool nonfinite_input = false;

    for (int i = 0; i < dof_num_; ++i) {
        const auto& lim = limits_[i];

        // 每个维度: kp=0, pos=1, kd=2, vel=3, torque=4
        for (int d = 0; d < 5; ++d) {
            float val = input(i, d);
            bool l1_fail = false;

            // ======== 1级检查 ========
            // NaN 检查
            if (!std::isfinite(val)) {
                nonfinite_input = true;
                l1_fail = true;
            } else {
                switch (d) {
                case 0: // kp
                    l1_fail = (val < 0.0f) || (val > lim.kp_hard_max);
                    break;
                case 1: // pos
                    l1_fail = (val < lim.pos_hard_lower) || (val > lim.pos_hard_upper);
                    break;
                case 2: // kd
                    l1_fail = (val < 0.0f) || (val > lim.kd_hard_max);
                    break;
                case 3: // vel
                    l1_fail = (std::abs(val) > lim.vel_hard_max);
                    break;
                case 4: // torque
                    l1_fail = (std::abs(val) > lim.tor_hard_max);
                    break;
                }
            }

            if (l1_fail) {
                // 值去重：仅在 (joint, dim) 变化或周期性帧间隔时打印
                if (i != last_l1_log_joint_ || d != last_l1_log_dim_ ||
                    l3_frame_counter_ % l2_log_interval_ == 0) {
                    logger_.error("[FATAL] L1 triggered: joint=%d, dim=%s, value=%f", i, dimName(d), val);
                    last_l1_log_joint_ = i;
                    last_l1_log_dim_ = d;
                }
                state_.l1_triggered = true;
                state_.l1_joint_idx = i;
                state_.l1_dim_idx = d;
                state_.l1_trigger_count++;
                // A suppressed damping transition still promises a clamped
                // packet. Finite L1 values must reach L2 clamping, including
                // the remaining dimensions of this joint. Never pass NaN/Inf.
                if (enable_damping_mode_ || !std::isfinite(val)) break;
            }

            // ======== 2级检查 ========
            bool l2_fail = false;
            float clamped = val;

            switch (d) {
            case 0: // kp
                l2_fail = (val < 0.0f || val > lim.kp_soft_max);
                if (l2_fail) clamped = std::clamp(val, 0.0f, lim.kp_soft_max);
                break;
            case 1: // pos
                if (val < lim.pos_soft_lower) { l2_fail = true; clamped = lim.pos_soft_lower; }
                else if (val > lim.pos_soft_upper) { l2_fail = true; clamped = lim.pos_soft_upper; }
                break;
            case 2: // kd
                l2_fail = (val < 0.0f || val > lim.kd_soft_max);
                if (l2_fail) clamped = std::clamp(val, 0.0f, lim.kd_soft_max);
                break;
            case 3: // vel
                if (val > lim.vel_soft_max) { l2_fail = true; clamped = lim.vel_soft_max; }
                else if (val < -lim.vel_soft_max) { l2_fail = true; clamped = -lim.vel_soft_max; }
                break;
            case 4: // torque
                if (val > lim.tor_soft_max) { l2_fail = true; clamped = lim.tor_soft_max; }
                else if (val < -lim.tor_soft_max) { l2_fail = true; clamped = -lim.tor_soft_max; }
                break;
            }

            if (l2_fail) {
                float raw_val = val;
                input(i, d) = clamped;
                state_.l2_counters[i][d]++;
                state_.l2_clamp_count++;
                if (l3_frame_counter_ % l2_log_interval_ == 0) {
                    logger_.warn("[WARN] L2 clamp: joint=%d, dim=%s, raw=%f, clamped=%f, counter=%d/%d",
                                 i, dimName(d), raw_val, clamped, state_.l2_counters[i][d], l2_threshold_);
                }
            } else {
                // 衰减计数器
                state_.l2_counters[i][d] = std::max(0, state_.l2_counters[i][d] - l2_decay_rate_);

                // ======== 3级检查（仅在 L2 未触发时检查）========
                bool l3_fail = false;
                switch (d) {
                case 0: l3_fail = (val > lim.kp_nominal_max); break;
                case 1: l3_fail = (val < lim.pos_nominal_lower) || (val > lim.pos_nominal_upper); break;
                case 2: l3_fail = (val > lim.kd_nominal_max); break;
                case 3: l3_fail = (std::abs(val) > lim.vel_nominal_max); break;
                case 4: l3_fail = (std::abs(val) > lim.tor_nominal_max); break;
                }

                if (l3_fail) {
                    if (l3_frame_counter_ % l3_log_interval_ == 0) {
                        logger_.info("[INFO] L3 boundary: joint=%d, dim=%s, value=%f", i, dimName(d), val);
                    }
                    state_.l3_log_count++;
                }
            }
        }
    }

    l3_frame_counter_++;

    // 帧级汇总判定
    if (state_.l1_triggered) {
        state_.l1_triggered = false;  // 重置供下次使用
        // L1 故障上报: severity = FATAL, resources 标明触发关节
        uint32_t resources = (state_.l1_joint_idx >= 0) ? (1u << state_.l1_joint_idx) : 0u;
        if (enable_fault_report_) {
            reportProtectionFault(resources, drdds::msg::FaultEvent_Constants::SEVERITY_FATAL);
        } else if (l3_frame_counter_ % l2_log_interval_ == 0) {
            logger_.warn("L1 fault report suppressed (enable_fault_report=false), resources=0x%08X", resources);
        }
        if (!enable_damping_mode_ && !nonfinite_input) {
            // 阻尼模式关闭：clamp 已在上面生效，返回 PASS 让调用方不进入阻尼模式
            if (l3_frame_counter_ % l2_log_interval_ == 0) {
                logger_.warn("L1 damping mode suppressed (enable_damping_mode=false), clamped cmd will be sent");
            }
            return ProtectionResult::PASS;
        }
        return ProtectionResult::L1_FATAL;
    }

    for (int i = 0; i < dof_num_; ++i) {
        for (int d = 0; d < 5; ++d) {
            if (state_.l2_counters[i][d] > l2_threshold_) {
                // L2 故障上报: severity = ERROR, resources 收集所有超限关节
                uint32_t resources = 0;
                for (int j = 0; j < dof_num_; ++j) {
                    for (int dd = 0; dd < 5; ++dd) {
                        if (state_.l2_counters[j][dd] > l2_threshold_) {
                            resources |= (1u << j);
                            break;
                        }
                    }
                }
                if (enable_fault_report_) {
                    reportProtectionFault(resources, drdds::msg::FaultEvent_Constants::SEVERITY_ERROR);
                } else if (l3_frame_counter_ % l2_log_interval_ == 0) {
                    logger_.warn("L2 fault report suppressed (enable_fault_report=false), resources=0x%08X", resources);
                }
                if (!enable_damping_mode_) {
                    // 阻尼模式关闭：clamp 已在上面生效，返回 PASS 让调用方不进入阻尼模式
                    if (l3_frame_counter_ % l2_log_interval_ == 0) {
                        logger_.warn("L2 damping mode suppressed (enable_damping_mode=false), clamped cmd will be sent");
                    }
                    return ProtectionResult::PASS;
                }
                return ProtectionResult::L2_ERROR;
            }
        }
    }

    // 正常通过时，如果之前有故障则上报 STOP
    if (last_protection_fault_code_ != 0) {
        if (enable_fault_report_) {
            reportProtectionFault(0, 0);
        } else if (l3_frame_counter_ % l2_log_interval_ == 0) {
            logger_.warn("L3 fault report suppressed (enable_fault_report=false), resources=0x%08X", 0);
        }
    }

    return ProtectionResult::PASS;
}

// ============================================================================
// dimName
// ============================================================================

void JointProtectionChecker::reportProtectionFault(uint32_t resources, uint8_t severity) {
    if (!fault_client_) return;

    // 资源位图未变化则不重复上报
    if (resources == last_protection_fault_code_) return;

    drdds::msg::FaultEvent fault_msg;
    fault_msg.code() = JNT_CMD_PROTECTION;
    fault_msg.grouped() = true;

    if (resources == 0) {
        fault_msg.type() = drdds::msg::FaultEvent_Constants::TYPE_STOP;
        fault_msg.severities() = {0};
    } else if (last_protection_fault_code_ == 0) {
        fault_msg.type() = drdds::msg::FaultEvent_Constants::TYPE_START;
        fault_msg.severities() = {severity};
    } else {
        fault_msg.type() = drdds::msg::FaultEvent_Constants::TYPE_START;  // Update existing fault
        fault_msg.severities() = {severity};
    }

    // resources 位图转字符串向量
    std::vector<std::string> res_vec;
    if (resources == 0) {
        res_vec.push_back("none");
    } else {
        for (int i = 0; i < 32; ++i) {
            if (resources & (1u << i)) {
                res_vec.push_back(std::to_string(i));
            }
        }
    }
    fault_msg.resources() = res_vec;

    fault_client_->pubEvent(fault_msg);
    last_protection_fault_code_ = resources;

    logger_.warn("Reported JNT_CMD_PROTECTION: type=%d, severity=%d, resources=0x%08X",
                 fault_msg.type(), severity, resources);
}

const char* JointProtectionChecker::dimName(int dim_idx) {
    switch (dim_idx) {
        case 0: return "kp";
        case 1: return "position";
        case 2: return "kd";
        case 3: return "velocity";
        case 4: return "torque";
        default: return "unknown";
    }
}
