#include "temperature_monitor.hpp"
#include <iostream>

using types::VecXf;

void TemperatureMonitor::init(int dof_num, const LevelMonitorConfig& cfg) {
    monitors_.resize(dof_num);
    accum_.resize(dof_num, 0.0);
    cnt_.resize(dof_num, 0);
    for (auto& mon : monitors_) {
        mon.configure(cfg);
    }
}

void TemperatureMonitor::sample(const VecXf& temps) {
    VecXf::Index idx;
    max_temp_ = temps.maxCoeff(&idx);
    max_temp_idx_ = static_cast<int>(idx);
    for (int i = 0; i < temps.size() && i < static_cast<int>(accum_.size()); i++) {
        accum_[i] += static_cast<double>(temps(i));
        cnt_[i]++;
    }
}

TempCheckResult TemperatureMonitor::check(bool hstop) {
    TempCheckResult r{};
    if (hstop) {
        for (size_t i = 0; i < accum_.size(); i++) { accum_[i] = 0.0; cnt_[i] = 0; }
        return r;
    }
    for (size_t i = 0; i < monitors_.size(); i++) {
        float filtered = (cnt_[i] > 0) ? static_cast<float>(accum_[i] / cnt_[i]) : 0.0f;
        monitors_[i].detect(filtered);
        auto sev = monitors_[i].current_severity();
        if (sev.has_value()) {
            if      (*sev == Severity::FATAL)  r.fatal_code  |= (1u << i);
            else if (*sev == Severity::ERROR)  r.error_code  |= (1u << i);
            else if (*sev == Severity::WARN)   r.warn_code   |= (1u << i);
            else if (*sev == Severity::NOTICE) r.notice_code |= (1u << i);
            else if (*sev == Severity::ALERT || *sev == Severity::EMERG)
                r.fatal_code |= (1u << i);  // ALERT/EMERG 比 ERROR 更严重，按 FATAL 处理
        }
    }
    for (size_t i = 0; i < accum_.size(); i++) { accum_[i] = 0.0; cnt_[i] = 0; }
    return r;
}

void TemperatureMonitor::print_raw(const VecXf& temps, const char* label) const {
    VecXf::Index mi;
    float min_val = temps.minCoeff(&mi);
    std::cout << label << " Temperture: " << temps.transpose() << "\n"
              << "Max&Min: " << max_temp_idx_ << " : " << max_temp_ << "  |  "
              << mi << " : " << min_val << std::endl;
}
