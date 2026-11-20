/**
 * @file level_monitor.hpp
 * @brief 多级阈值监控器 - 支持多阈值多等级的故障检测
 * @details 提供多级阈值监控能力，支持WARN/ERROR/FATAL等多个严重程度等级
 *
 * 配置输入: LevelMonitorConfig 结构体（格式无关，由 common.hpp 定义）
 * 只适配 strategy=thresholds 的情况：
 *   direction: upper/lower
 *   thresholds: {warn: 120, error: 150, fatal: 160}
 *   hysteresis: 10
 *   confirm_count: 3
 **/

#ifndef LEVEL_MONITOR_HPP
#define LEVEL_MONITOR_HPP

#include <algorithm>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "common.hpp"

// ============ 多级阈值监控器模板 ============
template <typename VT>
class LevelMonitor {
  static_assert(std::is_arithmetic_v<VT>, "VT must be an arithmetic type");

 public:
  struct SeverityLevel {
    Severity severity;
    VT enter_threshold;
    VT exit_threshold;
  };
  struct SeverityConfig {
    ThresholdType direction;
    std::vector<SeverityLevel> thresholds;
  };

 public:
  /**
   * @brief 配置监控器
   * @param cfg 配置结构
   * @return true 如果配置有效且设置成功（thresholds非空），false 如果thresholds为空
   */
  bool configure(const LevelMonitorConfig& cfg);

  /**
   * @brief 检测故障状态变化
   * @param value 当前观测值
   * @return 如果状态发生变化，返回事件信息；否则返回std::nullopt
   */
  std::optional<FaultEventInfo> detect(VT value);

  /**
   * @brief 获取当前严重程度
   */
  std::optional<Severity> current_severity() const { return current_severity_; }

 private:
  bool canEnter(VT value, VT threshold) const;
  bool canExit(VT value, VT threshold) const;
  std::optional<Severity> computeSeverity(VT value) const;

  SeverityConfig config_;
  std::optional<Severity> current_severity_;

  // 连续确认消抖机制：需连续 confirm_count_ 次采样满足同一目标才触发状态变更
  uint32_t confirm_count_ = 1;                ///< 连续确认次数, 默认1保持向后兼容
  std::optional<Severity> pending_severity_;  ///< 待确认的目标严重程度
  uint32_t confirm_counter_ = 0;              ///< 当前连续确认计数
};

// ============ 多级阈值监控器模板实现 ============
template <typename VT>
bool LevelMonitor<VT>::configure(const LevelMonitorConfig& cfg) {
  if (cfg.thresholds.empty()) {
    return false;
  }

  // 转换为内部 SeverityConfig
  SeverityConfig scfg;
  scfg.direction = cfg.direction;

  for (const auto& entry : cfg.thresholds) {
    VT enter = static_cast<VT>(entry.second);
    VT hysteresis_val = static_cast<VT>(cfg.hysteresis);
    VT exit_val =
        (cfg.direction == ThresholdType::GREATER_IS_WORSE) ? (enter - hysteresis_val) : (enter + hysteresis_val);
    scfg.thresholds.push_back({entry.first, enter, exit_val});
  }

  // 解析连续确认次数
  confirm_count_ = (cfg.confirm_count >= 1) ? cfg.confirm_count : 1;

  // 防御：迟滞为0且无确认消抖时，强制退化为debounce-only模式
  // 防止 enter/exit 阈值重叠时因依赖隐式排序保护引发临界振荡
  if (cfg.hysteresis <= 0 && confirm_count_ <= 1) {
    confirm_count_ = 3;
  }

  // 根据阈值类型排序：最严重的等级放在最前面
  auto& levels = scfg.thresholds;
  if (scfg.direction == ThresholdType::GREATER_IS_WORSE) {
    std::sort(levels.begin(), levels.end(),
              [](const auto& a, const auto& b) { return a.enter_threshold > b.enter_threshold; });
  } else {
    std::sort(levels.begin(), levels.end(),
              [](const auto& a, const auto& b) { return a.enter_threshold < b.enter_threshold; });
  }

  config_ = std::move(scfg);
  current_severity_ = std::nullopt;
  pending_severity_ = std::nullopt;
  confirm_counter_ = 0;
  return true;
}

template <typename VT>
std::optional<FaultEventInfo> LevelMonitor<VT>::detect(VT value) {
  auto target = computeSeverity(value);

  // 1. 目标与当前严重程度相同 — 无需任何状态变更，清除待确认状态
  if (target == current_severity_) {
    pending_severity_ = std::nullopt;
    confirm_counter_ = 0;
    return std::nullopt;
  }

  // 2. 目标与待确认方向不同 — 重置确认计数，开始新一轮确认
  if (target != pending_severity_) {
    pending_severity_ = target;
    confirm_counter_ = 1;
  } else {
    // 3. 目标与待确认方向相同 — 递增计数
    confirm_counter_++;
  }

  // 4. 确认计数未达到阈值 — 继续等待
  if (confirm_counter_ < confirm_count_) {
    return std::nullopt;
  }

  // 5. 确认达到阈值 — 状态变更生效，生成事件
  FaultEventInfo info{};
  if (!current_severity_.has_value()) {
    // 正常 → 异常: START
    info.fault_type = FaultType::START;
    info.new_severity = *target;
  } else if (!target.has_value()) {
    // 异常 → 正常: STOP
    info.fault_type = FaultType::STOP;
    info.old_severity = *current_severity_;
    info.new_severity = *current_severity_;
  } else {
    // 严重程度变化 — 使用 START
    info.fault_type = FaultType::START;
    info.new_severity = *target;
    info.old_severity = *current_severity_;
  }

  // 确认后更新当前状态并清除待确认
  current_severity_ = target;
  pending_severity_ = std::nullopt;
  confirm_counter_ = 0;
  return info;
}

template <typename VT>
bool LevelMonitor<VT>::canEnter(VT value, VT threshold) const {
  if (config_.direction == ThresholdType::GREATER_IS_WORSE) {
    return value >= threshold;
  } else {
    return value <= threshold;
  }
}

template <typename VT>
bool LevelMonitor<VT>::canExit(VT value, VT threshold) const {
  if (config_.direction == ThresholdType::GREATER_IS_WORSE) {
    return value <= threshold;
  } else {  // LESS_IS_WORSE
    return value >= threshold;
  }
}

template <typename VT>
std::optional<Severity> LevelMonitor<VT>::computeSeverity(VT value) const {
  std::optional<Severity> enter_severity = std::nullopt;
  const std::vector<SeverityLevel>& levels = config_.thresholds;
  for (const auto& lvl : levels) {
    if (canEnter(value, lvl.enter_threshold)) {
      enter_severity = lvl.severity;
      break;  // thresholds 已排序(enter_threshold)(方向=config_.direction)
    }
  }

  // 3.0 当前没有异常状态
  if (!current_severity_.has_value()) {
    return enter_severity;  // 正常到异常的进入
  }

  // 3.1 当前处于异常状态
  Severity current = *current_severity_;
  // 3.2 升级：enter_severity > current
  if (enter_severity.has_value() && *enter_severity > current) {
    return enter_severity;
  }

  // 3.3 无变化
  if (enter_severity.has_value() && *enter_severity == current) {
    return current_severity_;
  }

  // 3.4 降级 - 按严重性等级逐级检查（不按 threshold 排序顺序）
  auto current_it =
      std::find_if(levels.begin(), levels.end(), [&](const auto& lvl) { return lvl.severity == current; });
  if (current_it == levels.end()) {
    return std::nullopt;
  }

  // 检查是否仍在迟滞范围内，保持当前状态
  if (!canExit(value, static_cast<VT>(current_it->exit_threshold))) {
    return current_severity_;
  }

  // 从当前严重性下一级开始，逐级向下检查是否满足进入条件
  // 找到满足条件的最高可用低等级
  for (int severity_level = current - 1; severity_level >= 0; --severity_level) {
    auto candidate = std::find_if(levels.begin(), levels.end(),
                                  [severity_level](const auto& lvl) { return lvl.severity == severity_level; });

    if (candidate != levels.end() && canEnter(value, static_cast<VT>(candidate->enter_threshold))) {
      return candidate->severity;  // 降级到该等级
    }
  }

  // 没有低等级满足进入条件，恢复正常
  return std::nullopt;
}

#endif  // LEVEL_MONITOR_HPP
