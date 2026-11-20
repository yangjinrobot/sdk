#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "common.hpp"

/**
 * @brief DEM 服务端
 * @details 协调和管理所有故障诊断组件的容器 (Diagnostic Event Manager)
 *
 * 使用方式：构造即启动，析构即停止，一行代码完成初始化
 * @code
 *   Dem fc;  // 自动完成：加载配置 -> 初始化组件 -> 启动服务
 *   auto faults = fc.getAllFaults();
 * @endcode
 */
class DEM_API Dem {
 public:
  using FaultEvent = drdds::msg::FaultEvent;
  using SnapshotCallback = std::function<void(const std::vector<FaultEvent>&)>;

  /**
   * @brief 构造并启动 DEM 服务端
   * @details 自动完成：加载配置 -> 初始化组件 -> 启动服务
   */
  Dem();

  /**
   * @brief 析构并停止服务
   */
  ~Dem();

  /**
   * @brief 获取单例实例
   * @return Dem 单例引用
   * @details 线程安全，首次调用时创建实例
   * @note 单例模式：构造即启动，析构即停止
   *       推荐使用方式：auto& dm = Dem::instance();
   */
  static Dem& instance();

  // 禁止拷贝和移动
  Dem(const Dem&) = delete;
  Dem& operator=(const Dem&) = delete;
  Dem(Dem&&) = delete;
  Dem& operator=(Dem&&) = delete;

  // ==================== 故障查询接口 ====================

  /**
   * @brief 获取所有活跃故障
   * @return 故障事件列表
   */
  [[nodiscard]] std::vector<FaultEvent> getAllFaults() const;

  /**
   * @brief 按故障码查询
   * @param code 故障码
   * @return 故障事件列表
   */
  [[nodiscard]] std::vector<FaultEvent> getFaultsByCode(uint32_t code) const;

  /**
   * @brief 按严重程度查询
   * @param severity 严重程度
   * @return 故障事件列表
   */
  [[nodiscard]] std::vector<FaultEvent> getFaultsBySeverity(Severity severity) const;

  /**
   * @brief 按故障分组查询
   * @param group 故障分组
   * @return 故障事件列表
   */
  [[nodiscard]] std::vector<FaultEvent> getFaultsByGroup(FaultGroup group) const;

  // ==================== 生命周期/状态管理接口 ====================

  /**
   * @brief 是否已初始化并运行
   * @return true 已初始化
   */
  [[nodiscard]] bool isInitialized() const noexcept;

  /**
   * @brief 获取最近一次错误信息
   * @return 错误描述字符串
   */
  [[nodiscard]] std::string lastError() const noexcept;

  /**
   * @brief 关闭 DEM 服务
   */
  void shutdown() noexcept;

  // ==================== 回调注册接口 ====================

  /**
   * @brief 注册故障快照回调
   * @param callback 当故障列表变化时调用
   * @return 回调句柄
   */
  uint64_t onSnapshot(SnapshotCallback callback);

  /**
   * @brief 取消注册快照回调
   * @param handle 回调句柄
   */
  void offSnapshot(uint64_t handle);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
