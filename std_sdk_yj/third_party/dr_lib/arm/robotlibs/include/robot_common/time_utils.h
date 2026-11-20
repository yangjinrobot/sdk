#ifndef COMMON_TIME_UTILS_H
#define COMMON_TIME_UTILS_H

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

namespace robot_common {

/**
 * @brief 时间点类型定义
 */
using SteadyTimePoint = std::chrono::steady_clock::time_point;
using SystemTimePoint = std::chrono::system_clock::time_point;

/**
 * @brief 时间段类型定义
 */
using Nanoseconds = std::chrono::nanoseconds;
using Microseconds = std::chrono::microseconds;
using Milliseconds = std::chrono::milliseconds;
using Seconds = std::chrono::seconds;
using Minutes = std::chrono::minutes;
using Hours = std::chrono::hours;

/**
 * @brief 时间工具类
 * 提供各种时间相关的静态方法
 */
class TimeUtils {
 public:
  // =========================================================================
  // 获取当前时间
  // =========================================================================

  /**
   * @brief 获取当前时间戳（毫秒）- 单调时钟
   */
  static uint64_t GetSteadyMilliseconds();

  /**
   * @brief 获取当前系统时间戳（毫秒）- 自Unix纪元
   */
  static uint64_t GetSystemMilliseconds();

  /**
   * @brief 获取当前时间点
   */
  static SteadyTimePoint GetSteadyNow();
  static SystemTimePoint GetSystemNow();

  // =========================================================================
  // 时间格式化
  // =========================================================================

  /**
   * @brief 时间戳转字符串
   * @param timestamp_ms 毫秒时间戳
   * @param format 格式字符串，默认 "%Y-%m-%d %H:%M:%S"
   * @param with_ms 是否包含毫秒
   */
  static std::string FormatTimestamp(uint64_t timestamp_ms, const std::string& format = "%Y-%m-%d %H:%M:%S",
                                     bool with_ms = true);

  /**
   * @brief 获取当前时间字符串
   */
  static std::string GetCurrentTimeString(const std::string& format = "%Y-%m-%d %H:%M:%S", bool with_ms = true);

  // =========================================================================
  // 时间计算
  // =========================================================================

  /**
   * @brief 计算两个时间点之间的差值（毫秒）
   */
  static int64_t DiffMilliseconds(const SteadyTimePoint& start, const SteadyTimePoint& end);

  // =========================================================================
  // 睡眠函数
  // =========================================================================

  /**
   * @brief 睡眠指定时长（模板函数，支持任意时间单位）
   */
  template <typename Rep, typename Period>
  static void SleepFor(std::chrono::duration<Rep, Period> duration) {
    std::this_thread::sleep_for(duration);
  }

 private:
  TimeUtils() = delete;
  ~TimeUtils() = delete;
};

/**
 * @brief 计时器类
 * 用于测量代码执行时间
 */
class Timer {
 public:
  Timer();
  explicit Timer(const std::string& name);

  /**
   * @brief 启动计时器
   */
  void Start();

  /**
   * @brief 停止计时器
   */
  void Stop();

  /**
   * @brief 重置计时器
   */
  void Reset();

  /**
   * @brief 重新开始（Reset + Start）
   */
  void Restart();

  /**
   * @brief 获取经过时间（纳秒）
   */
  uint64_t ElapsedNanoseconds() const;

  /**
   * @brief 获取经过时间（微秒）
   */
  uint64_t ElapsedMicroseconds() const;

  /**
   * @brief 获取经过时间（毫秒）
   */
  uint64_t ElapsedMilliseconds() const;

  /**
   * @brief 获取经过时间（秒，浮点数）
   */
  double ElapsedSeconds() const;

  /**
   * @brief 检查计时器是否正在运行
   */
  bool IsRunning() const;

  /**
   * @brief 获取计时器名称
   */
  const std::string& GetName() const;

 private:
  std::string name_;
  SteadyTimePoint start_time_;
  Nanoseconds accumulated_{0};
  bool running_ = false;
};

/**
 * @brief 速率控制器
 * 用于控制循环执行频率
 */
class RateController {
 public:
  /**
   * @brief 构造函数
   * @param frequency_hz 目标频率（Hz）
   */
  explicit RateController(double frequency_hz);

  /**
   * @brief 设置目标频率
   */
  void SetFrequency(double frequency_hz);

  /**
   * @brief 获取目标频率
   */
  double GetFrequency() const;

  /**
   * @brief 获取目标周期（毫秒）
   */
  uint64_t GetPeriodMs() const;

  /**
   * @brief 睡眠以维持目标频率
   * 应在每个循环迭代结束时调用
   */
  void Sleep();

  /**
   * @brief 重置计时器
   */
  void Reset();

  /**
   * @brief 获取实际频率
   */
  double GetActualFrequency() const;

  /**
   * @brief 检查是否发生周期超时（循环执行时间超过周期）
   */
  bool IsOverrun() const;

  /**
   * @brief 获取上一次循环的实际执行时间（毫秒）
   */
  uint64_t GetLastCycleTimeMs() const;

 private:
  double target_frequency_;
  Nanoseconds period_;
  SteadyTimePoint last_time_;
  SteadyTimePoint cycle_start_;
  uint64_t cycle_count_ = 0;
  bool overrun_ = false;
  Nanoseconds last_cycle_time_{0};
};

/**
 * @brief 作用域计时器
 * 自动在构造时开始计时，析构时打印耗时
 */
class ScopedTimer {
 public:
  explicit ScopedTimer(const std::string& name, bool print_on_destruct = true);
  ~ScopedTimer();

  // 禁止拷贝和移动
  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

  /**
   * @brief 获取当前已经过时间（毫秒）
   */
  uint64_t ElapsedMilliseconds() const;

 private:
  std::string name_;
  SteadyTimePoint start_time_;
  bool print_on_destruct_;
};

/**
 * @brief 超时检测器
 */
class TimeoutDetector {
 public:
  /**
   * @brief 构造函数
   * @param timeout_ms 超时时间（毫秒）
   */
  explicit TimeoutDetector(uint64_t timeout_ms);

  /**
   * @brief 重置（喂狗）
   */
  void Reset();

  /**
   * @brief 检查是否超时
   */
  bool IsTimeout() const;

  /**
   * @brief 获取距离超时的剩余时间（毫秒）
   */
  uint64_t GetRemainingMs() const;

  /**
   * @brief 设置超时时间
   */
  void SetTimeout(uint64_t timeout_ms);

  /**
   * @brief 获取超时时间
   */
  uint64_t GetTimeout() const;

 private:
  uint64_t timeout_ms_;
  SteadyTimePoint last_reset_;
};

}  // namespace robot_common

#endif  // COMMON_TIME_UTILS_H
