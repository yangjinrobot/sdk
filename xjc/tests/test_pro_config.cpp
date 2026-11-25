#include "pro_sdk_config.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>

static void Require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
int main() {
    namespace fs = std::filesystem;
    const fs::path path = fs::temp_directory_path() / ("pro-config-" + std::to_string(::getpid()) + ".toml");
    try {
        ::setenv("MOTION_MASTER_CONFIG", path.c_str(), 1);
        { std::ofstream out(path); out << "[pro_sdk]\nudp_enabled = false\nudp_local_ip = \"127.0.0.1\"\nudp_local_port = 18765\n"; }
        const auto configured = LoadProSdkConfig();
        Require(!configured.udp_enabled && configured.udp_local_ip == "127.0.0.1" &&
                configured.udp_local_port == 18765 && configured.udp_remote_port == 9999,
                "explicit configuration or source defaults changed");
        { std::ofstream out(path); out << "[pro_sdk]\nudp_local_port = 0\n"; }
        bool invalid_port = false;
        try { (void)LoadProSdkConfig(); } catch (const std::runtime_error&) { invalid_port = true; }
        Require(invalid_port, "invalid UDP port accepted");
        fs::remove(path);
        bool missing = false;
        try { (void)LoadProSdkConfig(); } catch (const std::runtime_error&) { missing = true; }
        Require(missing, "missing explicit configuration silently ignored");
        ::unsetenv("MOTION_MASTER_CONFIG");
        std::cout << "PRO configuration contracts passed\n";
        return 0;
    } catch (const std::exception& error) {
        fs::remove(path);
        std::cerr << error.what() << '\n';
        return 1;
    }
}
