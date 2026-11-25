#pragma once

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace pro_policy {

inline std::string ResolveModelPath(const std::string& relative_path) {
    namespace fs = std::filesystem;
    const auto require_file = [&](const fs::path& root) {
        const fs::path file = root / relative_path;
        if (!fs::is_regular_file(file)) {
            throw std::runtime_error("CR1PRO policy file does not exist: " + file.string());
        }
        return fs::canonical(file).string();
    };
    if (const char* configured = std::getenv("MOTION_MASTER_POLICY_DIR")) {
        if (!*configured) throw std::runtime_error("MOTION_MASTER_POLICY_DIR is empty");
        return require_file(fs::path(configured));
    }
    const fs::path installed("/opt/robot/share/motion_master/policy/cr1pro");
    if (fs::is_regular_file(installed / relative_path)) return require_file(installed);
#ifdef MOTION_MASTER_PRO_POLICY_DIR
    const fs::path source(MOTION_MASTER_PRO_POLICY_DIR);
#else
    const fs::path source = fs::path(__FILE__).parent_path().parent_path() / "policy";
#endif
    return require_file(source);
}

}  // namespace pro_policy
