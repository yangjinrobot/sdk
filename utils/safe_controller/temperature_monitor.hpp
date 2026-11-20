#pragma once
#include "dem/level_monitor.hpp"
#include "common_types.h"
#include <vector>
#include <cstdint>

struct TempCheckResult {
    uint32_t fatal_code  = 0;
    uint32_t error_code  = 0;
    uint32_t warn_code   = 0;
    uint32_t notice_code = 0;
};

class TemperatureMonitor {
public:
    void init(int dof_num, const LevelMonitorConfig& cfg);

    // 1ms 采样：累加温度
    void sample(const types::VecXf& temps);

    // 1s 判断：取均值 → detect → 返回严重度位图，内部清零累加器
    TempCheckResult check(bool hstop);

    // 5s 打印原始温度到 stdout
    void print_raw(const types::VecXf& temps, const char* label) const;

    float max_temp()    const { return max_temp_; }
    int   max_temp_idx() const { return max_temp_idx_; }

private:
    std::vector<LevelMonitor<float>> monitors_;
    std::vector<double> accum_;
    std::vector<int>    cnt_;
    float max_temp_    = 0;
    int   max_temp_idx_ = 0;
};
