#pragma once

#include <cstdlib>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace pro_policy {

std::string ResolveModelPath(const std::string& relative_path);

}  // namespace pro_policy
