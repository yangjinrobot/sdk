/**
 * @file joint_protection.hpp
 * @brief SetJointCommand 三级保护检查器
 * @author deeprobotics
 * @version 1.0
 * @date 2026-04-08
 *
 * @copyright Copyright (c) 2026  DeepRobotics
 *
 */

#pragma once

#include "common_types.h"
#include "dr_logger.hpp"
#include "dem/fault_client.hpp"
#include "toml/toml.hpp"

#include <vector>
#include <array>
#include <cstdint>
#include <memory>

// 项目专用故障码
static constexpr int JNT_CMD_PROTECTION = 0x8022;  // 关节指令保护（L1=FATAL, L2=ERROR）

class ControlParameters;

using namespace types;

/**
 * @brief 保护检查结果枚举
 */
enum class ProtectionResult {
    PASS,       // 正常通过
    L1_FATAL,   // 1级硬限位触发
    L2_ERROR    // 2级软限位连续超限触发
};

/**
 * @brief 每个关节的三级保护阈值
 */
struct JointProtectionLimits {
    // 1级硬限位
    float kp_hard_max;
    float pos_hard_lower, pos_hard_upper;
    float kd_hard_max;
    float vel_hard_max;
    float tor_hard_max;
    // 2级软限位
    float kp_soft_max;
    float pos_soft_lower, pos_soft_upper;
    float kd_soft_max;
    float vel_soft_max;
    float tor_soft_max;
    // 3级理论边界
    float kp_nominal_max;
    float pos_nominal_lower, pos_nominal_upper;
    float kd_nominal_max;
    float vel_nominal_max;
    float tor_nominal_max;
};

/**
 * @brief 运行时保护状态
 */
struct JointProtectionState {
    std::vector<std::array<int, 5>> l2_counters;  // [关节数][5维度]
    bool l1_triggered = false;
    int l1_joint_idx = -1;
    int l1_dim_idx = -1;
    uint64_t l1_trigger_count = 0;
    uint64_t l2_clamp_count = 0;
    uint64_t l3_log_count = 0;

    /**
     * @brief 初始化/重置计数器
     * @param dof_num 关节数
     */
    void reset(int dof_num);
};

/**
 * @brief SetJointCommand 三级保护检查器
 */
class JointProtectionChecker {
public:
    /**
     * @brief 初始化保护检查器
     * @param dof_num 关节自由度数
     * @param cp_ptr 控制参数指针
     * @param config TOML 配置
     */
    void init(int dof_num, const ControlParameters* cp_ptr, const toml::value& config, FaultClient* fault_client = nullptr);

    /**
     * @brief 执行三级保护检查
     * @param input 关节指令矩阵 [dof x 5]，列序: kp, pos, kd, vel, torque
     * @return 保护结果
     */
    ProtectionResult check(Eigen::Matrix<float, Eigen::Dynamic, 5>& input);

    /**
     * @brief 是否启用保护
     */
    bool isEnabled() const;

    /**
     * @brief 获取阻尼 kd 配置值
     */
    float getDampingKd() const { return damping_kd_; }

private:
    bool enabled_ = false;
    int dof_num_ = 0;
    int l2_threshold_ = 10;
    int l2_decay_rate_ = 1;
    float damping_kd_ = 10.0f;
    float margin_l1_ = 0.5f;
    int l3_log_interval_ = 100;
    int l2_log_interval_ = 100;   // L2 warn 日志帧间隔（避免刷屏）
    int l3_frame_counter_ = 0;

    // L1 日志去重：相同 (joint, dim) 不重复打印
    int last_l1_log_joint_ = -1;
    int last_l1_log_dim_ = -1;

    std::vector<JointProtectionLimits> limits_;  // 每个关节一份
    JointProtectionState state_;
    DrLogger logger_{"JointProtection"};

    bool enable_fault_report_ = true;   // 是否上报故障事件（false 时仍记录日志）
    bool enable_damping_mode_ = true;    // 是否允许 L2 触发阻尼模式（false 时 L2 做 clamp 但返回 PASS）

    // 故障上报（非拥有指针，由外部 SafeController 提供）
    FaultClient* fault_client_ = nullptr;
    uint32_t last_protection_fault_code_ = 0;

    /**
     * @brief 上报关节指令保护故障
     * @param resources 触发关节的位图
     * @param severity 严重级别
     */
    void reportProtectionFault(uint32_t resources, uint8_t severity);

    /**
     * @brief 从 ControlParameters 和 TOML 配置初始化各关节阈值
     */
    void initLimitsFromConfig(const ControlParameters* cp_ptr, const toml::value& config);

    /**
     * @brief 获取维度名称（用于日志）
     * @param dim_idx 维度索引 0~4
     * @return 维度名称字符串
     */
    static const char* dimName(int dim_idx);
};
