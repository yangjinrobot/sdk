#pragma once
#include <cstdlib>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "toml/toml.hpp"

struct ProSdkConfig {
    bool udp_enabled = true;
    std::string udp_local_ip = "10.21.33.103";
    int udp_local_port = 8888;
    std::string udp_remote_ip = "10.21.33.7";
    int udp_remote_port = 9999;
};

inline ProSdkConfig LoadProSdkConfig() {
    ProSdkConfig result;
    const char* override_path = std::getenv("MOTION_MASTER_CONFIG");
    const std::string explicit_path = override_path ? override_path : "";
    const std::vector<std::string> paths = explicit_path.empty()
        ? std::vector<std::string>{"/var/opt/robot/conf/motion_master/config.toml", "conf/config.toml"}
        : std::vector<std::string>{explicit_path};
    for (const auto& path : paths) {
        if (!std::ifstream(path).good()) {
            if (!explicit_path.empty()) throw std::runtime_error("MOTION_MASTER_CONFIG does not exist: " + path);
            continue;
        }
        const auto config = toml::parse(path);
        if (config.contains("pro_sdk")) {
            const auto& pro = toml::find(config, "pro_sdk");
            result.udp_enabled = toml::find_or<bool>(pro, "udp_enabled", result.udp_enabled);
            result.udp_local_ip = toml::find_or<std::string>(pro, "udp_local_ip", result.udp_local_ip);
            result.udp_local_port = toml::find_or<int>(pro, "udp_local_port", int(result.udp_local_port));
            result.udp_remote_ip = toml::find_or<std::string>(pro, "udp_remote_ip", result.udp_remote_ip);
            result.udp_remote_port = toml::find_or<int>(pro, "udp_remote_port", int(result.udp_remote_port));
        }
        break;
    }
    if (result.udp_local_port < 1 || result.udp_local_port > 65535 ||
        result.udp_remote_port < 1 || result.udp_remote_port > 65535)
        throw std::runtime_error("PRO UDP ports must be between 1 and 65535");
    return result;
}
