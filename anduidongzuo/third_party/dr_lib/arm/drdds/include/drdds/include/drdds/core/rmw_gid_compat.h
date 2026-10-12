/**
 * @file rmw_gid_compat.h
 * @brief 与 ROS2 rmw_gid_t / create_rmw_gid 规则一致，用于 client/server 侧基于 GID 的校验与互操作。
 *
 * 对应 ROS2 定义：
 *   typedef struct rmw_gid_s {
 *     const char * implementation_identifier;
 *     uint8_t data[RMW_GID_STORAGE_SIZE];
 *   } rmw_gid_t;
 *   rmw_gid_t create_rmw_gid(const char * identifier, const GUID_t & guid);
 */
#ifndef DRDDS_CORE_RMW_GID_COMPAT_H_
#define DRDDS_CORE_RMW_GID_COMPAT_H_

#include <fastrtps/rtps/common/Guid.h>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace drdds {

#ifndef RMW_GID_STORAGE_SIZE
#define RMW_GID_STORAGE_SIZE 24
#endif

constexpr size_t kRmwGidStorageSize = RMW_GID_STORAGE_SIZE;

/** Fast DDS GUID_t 为 16 字节 (GuidPrefix 12 + EntityId 4)，与 rmw_fastrtps 一致 */
constexpr size_t kFastDdsGuidSize = 12 + 4;

/**
 * 与 rmw_gid_t 同布局的 GID 结构，便于与 ROS2 rmw_gid_t 互用（如 memcpy 或 reinterpret_cast）
 */
struct RmwGid
{
  const char* implementation_identifier;
  uint8_t data[RMW_GID_STORAGE_SIZE];
};

/**
 * 将 Fast DDS GUID_t 写入 data 区（与 rmw_fastrtps copy_from_fastrtps_guid_to_byte_array 一致）
 */
inline void copy_from_fastrtps_guid_to_byte_array(
    const eprosima::fastrtps::rtps::GUID_t& guid,
    uint8_t* data)
{
  static_assert(
      sizeof(eprosima::fastrtps::rtps::GUID_t) <= RMW_GID_STORAGE_SIZE,
      "RMW_GID_STORAGE_SIZE insufficient to store the fastrtps GUID_t.");
  std::memcpy(data, guid.guidPrefix.value, 12);
  std::memcpy(data + 12, guid.entityId.value, 4);
  if (kRmwGidStorageSize > kFastDdsGuidSize) {
    std::memset(data + kFastDdsGuidSize, 0, kRmwGidStorageSize - kFastDdsGuidSize);
  }
}

/**
 * 从 data 区读出为 Fast DDS GUID_t（与 rmw_fastrtps 从 rmw_gid_t.data 解析一致）
 */
inline void copy_from_byte_array_to_fastrtps_guid(
    const uint8_t* data,
    eprosima::fastrtps::rtps::GUID_t& guid)
{
  std::memcpy(guid.guidPrefix.value, data, 12);
  std::memcpy(guid.entityId.value, data + 12, 4);
}

/**
 * 按 ROS2 create_rmw_gid 规则构造 GID：identifier + GUID 写入 data
 */
inline RmwGid create_rmw_gid(
    const char* identifier,
    const eprosima::fastrtps::rtps::GUID_t& guid)
{
  RmwGid rmw_gid = {};
  rmw_gid.implementation_identifier = identifier;
  copy_from_fastrtps_guid_to_byte_array(guid, rmw_gid.data);
  return rmw_gid;
}

/**
 * 比较两个 GUID_t 是否相等（与 ROS2 GID 比较语义一致）
 */
inline bool guid_equals(
    const eprosima::fastrtps::rtps::GUID_t& a,
    const eprosima::fastrtps::rtps::GUID_t& b)
{
  return a == b;
}

/**
 * 比较两个 RmwGid 的 data 区是否一致（前 16 字节为 GUID）
 */
inline bool rmw_gid_data_equals(const uint8_t* a, const uint8_t* b)
{
  return std::memcmp(a, b, kFastDdsGuidSize) == 0;
}

}  // namespace drdds

#endif /* DRDDS_CORE_RMW_GID_COMPAT_H_ */
