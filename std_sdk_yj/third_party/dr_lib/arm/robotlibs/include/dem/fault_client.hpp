/**
 * @file fault_client.hpp
 * @brief 故障客户端（支持单例和自定义实例）
 * @details 封装故障上报和心跳管理，构造时自动完成初始化
 *
 * 使用流程：
 * - 单例：`auto& client = FaultClient::instance();`
 * - 自定义：`FaultClient client("process");`
 *
 * 多 SOC 支持：
 * - source 格式："process#soc_id"（如 "motion_master#soc0"）
 * - 配置文件中的 source 只需填写 process
 * - FaultClient 自动获取 soc_id 并拼接为实际 source
 * - 使用 '#' 作为分隔符，避免与进程名中常见的下划线冲突
 *
 * 构造函数说明：
 * - 公开构造函数：客户端集成使用，自动获取 soc_id，使用默认配置路径
 * - 私有构造函数：单元测试使用，可自定义 soc_id 和配置路径
 */

#ifndef FAULT_CLIENT_HPP
#define FAULT_CLIENT_HPP

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "common.hpp"
#include "drdds/core/common_type.h"

struct FaultClientImpl;

// ============================================================================
// FaultClient 类声明
// ============================================================================

class DEM_API FaultClient {
 public:
  using FaultEvent = drdds::msg::FaultEvent;

  /**
   * @brief 错误码枚举
   */
  enum class ErrorCode : int32_t {
    Success = 0,
    // 初始化相关
    NotInitialized = -1,
    ConfigLoadFailed = -2,
    // 发布相关
    PublishFailed = -10,
    InvalidEvent = -11,
    EventNotConfigured = -12,
    RateLimited = -13,
    // 通用
    Unknown = -100
  };

  /**
   * @brief 客户端状态枚举
   */
  enum class Status { NotReady, Ready };

  /**
   * @brief 客户端信息结构体
   */
  struct ClientInfo {
    std::string process;
    std::string soc_id;
    std::string source;
  };

  /**
   * @brief 公开构造函数（客户端集成使用）
   * @param process 本进程名称（用于填充 source 字段）
   * @details 构造时自动完成：获取 soc_id、创建 DDS 通道、加载配置、启动心跳
   * @note 最终的 source 格式为 "process#soc_id"
   */
  explicit FaultClient(const std::string& process);

  /**
   * @brief 析构函数
   */
  ~FaultClient();

  FaultClient(const FaultClient&) = delete;
  FaultClient& operator=(const FaultClient&) = delete;
  FaultClient(FaultClient&&) noexcept = delete;
  FaultClient& operator=(FaultClient&&) noexcept = delete;

  /**
   * @brief 获取客户端当前状态
   * @return 状态枚举值
   */
  [[nodiscard]] Status status() const;

  /**
   * @brief 获取状态描述信息
   * @return 状态对应的描述字符串
   */
  [[nodiscard]] std::string statusMessage() const;

  /**
   * @brief 发布故障事件
   * @param event 故障事件
   * @return 错误码，0 表示成功
   */
  int32_t pubEvent(const FaultEvent& event);

  /**
   * @brief 批量发布故障事件
   * @param events 故障事件列表
   * @return 错误码，0 表示成功
   */
  int32_t pubEvents(const std::vector<FaultEvent>& events);

  /**
   * @brief 查找指定故障码的配置
   * @param code 故障码
   * @return 配置项，不存在则返回 std::nullopt
   */
  [[nodiscard]] std::optional<FaultCfgItem> findConfig(uint32_t code) const;

  /**
   * @brief 获取所有故障配置
   * @return 所有故障配置数组
   */
  [[nodiscard]] std::vector<FaultCfgItem> getAllConfigs() const;

  /**
   * @brief 获取客户端信息
   * @return 包含 process、soc_id、source 的信息结构体
   */
  [[nodiscard]] ClientInfo info() const;

 private:
  // 测试辅助接口（friend class FaultClientTestFixture 可访问）
  explicit FaultClient(const std::string& process, const std::string& soc_id, const std::string& config_path);
  size_t getActiveFaultsCount() const;
  friend class FaultClientTestFixture;

 private:
  std::unique_ptr<FaultClientImpl> impl_;
};

#endif  // FAULT_CLIENT_HPP
