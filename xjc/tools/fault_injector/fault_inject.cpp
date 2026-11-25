// ============================================================================
// fault_inject.cpp
//
// 离线调试工具：通过 DDS 向 motion_master 注入模拟故障，验证状态机故障响应。
//
// 原理：
//   向 rt/motion_master/fault_inject 话题发布 String 命令，
//   SafeController 订阅该话题并根据命令设置注入故障掩码（OR 进 GetFaultMask），
//   触发状态机的分级降级逻辑（上半身保护 / JointDamping / 恢复观测）。
//
// 用法：
//   ./fault_inject pos_over_limit <joint_mask>   设置关节位置超限
//   ./fault_inject nan <joint_mask>               设置关节数据 NaN
//   ./fault_inject update_err <joint_mask>        设置关节数据不更新
//   ./fault_inject driver_status <joint_mask>     设置驱动器状态异常
//   ./fault_inject clear <type>                   清除单类故障注入
//   ./fault_inject clear all                      清除全部注入故障
//
// joint_mask 格式：支持十进制或十六进制（0x 前缀），每 bit 对应一个关节编号。
//
// 示例（CR1PRO 31DOF）：
//   # 腰部关节(0-2) 位置超限 → WAIST_Z → RLControl 清零上半身
//   ./fault_inject pos_over_limit 0x4       # bit2 → waist_z
//
//   # 右臂关节(10-16) 位置超限 → UPPER_LIMB → RLControl 清零上半身
//   ./fault_inject pos_over_limit 0x1FC00   # bits 10-16
//
//   # 左腿关节(17-22) 位置超限 → LEG → JointDamping
//   ./fault_inject pos_over_limit 0x7E0000  # bits 17-22
//
//   # 脖子关节(29-30) 位置超限 → HEAD → RLControl 仅告警
//   ./fault_inject pos_over_limit 0x60000000 # bits 29,30
//
//   # 清除所有注入故障
//   ./fault_inject clear all
//
// 编译（x86 本地，依赖 DDS libs）：
//   见同目录 build.sh
// ============================================================================

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <chrono>

#include "drdds/core/drdds_manager.h"
#include "drdds/core/common_type.h"

namespace {

void printUsage(const char* prog) {
    std::printf("Usage: %s <command> [args]\n\n", prog);
    std::printf("Commands:\n");
    std::printf("  SET <type> <mask>    注入故障 (type: pos_over_limit|nan|update_err|driver_status)\n");
    std::printf("  CLEAR <type>         清除单类故障\n");
    std::printf("  CLEAR ALL            清除全部注入故障\n\n");
    std::printf("  mask: 十六进制 (0xNNN) 或十进制，每 bit 对应一个关节编号\n\n");
    std::printf("Examples:\n");
    std::printf("  %s pos_over_limit 0x4          # waist_z 位置超限\n", prog);
    std::printf("  %s pos_over_limit 0x1FC00      # 右臂 位置超限\n", prog);
    std::printf("  %s pos_over_limit 0x7E0000     # 左腿 位置超限\n", prog);
    std::printf("  %s nan 0x1000                  # joint12 NaN\n", prog);
    std::printf("  %s driver_status 0xFF          # 驱动器0-7异常\n", prog);
    std::printf("  %s clear all                   # 清除全部\n", prog);
    std::printf("\nJoint index reference (CR1PRO 31DOF):\n");
    std::printf("  waist[0-2]  larm[3-9]  rarm[10-16]  lleg[17-22]  rleg[23-28]  neck[29-30]\n");
}

uint32_t parseMask(const char* str) {
    if (!str || !*str) return 0;
    // auto-detect hex (0x prefix) vs decimal
    if (str[0] == '0' && (str[1] == 'x' || str[1] == 'X')) {
        return static_cast<uint32_t>(std::strtoul(str + 2, nullptr, 16));
    }
    return static_cast<uint32_t>(std::strtoul(str, nullptr, 0));
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        printUsage(argv[0]);
        return 1;
    }

    std::string action = argv[1];

    // 将命令行参数组装为 topic 消息格式: "SET <type> <mask>" 或 "CLEAR <type>"
    std::string cmd;
    if (action == "clear" || action == "CLEAR") {
        if (argc >= 3) {
            std::string type = argv[2];
            if (type == "all" || type == "ALL") {
                cmd = "CLEAR ALL";
            } else {
                cmd = "CLEAR " + type;
            }
        } else {
            std::printf("Error: CLEAR requires <type> or ALL\n");
            printUsage(argv[0]);
            return 1;
        }
    } else if (argc >= 3) {
        // 直接传 type + mask: ./fault_inject pos_over_limit 0x4
        std::string type = argv[1];
        uint32_t mask = parseMask(argv[2]);

        // 验证 type
        if (type != "pos_over_limit" && type != "nan" &&
            type != "update_err" && type != "driver_status") {
            std::printf("Error: unknown fault type '%s'\n", type.c_str());
            printUsage(argv[0]);
            return 1;
        }

        char buf[64];
        std::snprintf(buf, sizeof(buf), "0x%X", mask);
        cmd = "SET " + type + " " + std::string(buf);
    } else {
        printUsage(argv[0]);
        return 1;
    }

    std::printf("[fault_inject] publishing: '%s'\n", cmd.c_str());

    // 初始化 DDS participant（必须在创建 ChannelString 之前）
    DrDDSManager::Init(0, "eth0");

    // 创建 DDS String 发布者
    ChannelString publisher("/motion_master/fault_inject", 0);

    // 等待匹配（对端 motion_master 订阅者）
    std::printf("[fault_inject] waiting for subscriber match...\n");
    int wait_ms = 0;
    const int kMaxWaitMs = 2000;
    while (publisher.GetMatchedCount() == 0 && wait_ms < kMaxWaitMs) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        wait_ms += 100;
    }

    if (publisher.GetMatchedCount() == 0) {
        std::printf("[fault_inject] WARNING: no subscriber matched after %d ms, "
                     "publishing anyway (motion_master may not be running)\n",
                     kMaxWaitMs);
    } else {
        std::printf("[fault_inject] subscriber matched (%d)\n",
                     publisher.GetMatchedCount());
    }

    // 发布命令
    publisher.data_->data(cmd);
    bool ok = publisher.Write();
    if (ok) {
        std::printf("[fault_inject] SUCCESS: command sent\n");
    } else {
        std::printf("[fault_inject] FAILED: Write() returned false\n");
        return 1;
    }

    // 短暂等待确保发送完成
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    return ok ? 0 : 1;
}
