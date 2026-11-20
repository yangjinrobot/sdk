/**
 * @file neck_takeover.hpp
 * @brief 脖子接管控制工具类：toml 配置加载、滑动滤波、SetJointCommand
 *        路径上的零侵入覆盖。RobotInterface 聚合持有，hardware/sim
 *        接口在 SetJointCommand 入口调用 Apply()。
 * @author zhenglinqiang
 */
#ifndef NECK_TAKEOVER_HPP_
#define NECK_TAKEOVER_HPP_

#include <atomic>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include "toml/toml.hpp"

namespace utils {

class NeckTakeover {
public:
    void SetEnabled(bool en){
        const bool prev = enabled_.exchange(en, std::memory_order_acq_rel);
        // 从 false 跳变为 true 时，重置 kp/kd 边沿检测状态，以便下一次 kp/kd
        // 激活时重新以关节实际位置锚定，避免从历史残留的 filt 起步。
        if (en && !prev) {
            prev_kp_kd_active_.store(false, std::memory_order_release);
        }
    }
    bool IsEnabled() const { return enabled_.load(std::memory_order_acquire); }

    /// 脖子机械锁死开关：true 时 Apply 会将 neck 关节指令整行（kp/pos/kd/vel/tau）清零
    void SetLocked(bool en){ locked_.store(en, std::memory_order_release); }
    bool IsLocked() const  { return locked_.load(std::memory_order_acquire); }

    void SetCmd(float yaw_cmd, float pitch_cmd){
        yaw_cmd_.store(yaw_cmd,     std::memory_order_release);
        pitch_cmd_.store(pitch_cmd, std::memory_order_release);
    }

    void SetParams(float yaw_range_rad, float pitch_range_rad,
                   float max_step_rad, float deadzone){
        yaw_range_rad_   = std::max(0.f, yaw_range_rad);
        pitch_range_rad_ = std::max(0.f, pitch_range_rad);
        max_step_rad_    = std::max(0.f, max_step_rad);
        deadzone_        = std::clamp(deadzone, 0.f, 1.f);
    }

    /// 加载 [neck_takeover_control] 段；path 为空时探测默认路径，
    /// 缺文件/缺段时保留当前默认值。neck_dof_num 仅用于日志。
    void LoadFromToml(const std::string& path = "", int neck_dof_num = -1){
        std::string toml_path = path;
        if (toml_path.empty()) {
            for (const char* candidate : {
                "/var/opt/robot/conf/motion_master/config.toml",
                "conf/config.toml"}) {
                if (std::filesystem::exists(candidate)) { toml_path = candidate; break; }
            }
        }
        if (toml_path.empty() || !std::filesystem::exists(toml_path)) {
            std::cout << "[neck_takeover_control] config not found, using defaults"
                      << " (neck_dof_num=" << neck_dof_num << ")" << std::endl;
            return;
        }
        try {
            auto cfg = toml::parse(toml_path);
            if (!cfg.contains("neck_takeover_control")) {
                std::cout << "[neck_takeover_control] section missing in " << toml_path
                          << ", using defaults" << std::endl;
                return;
            }
            auto deg2rad = [](double d){ return d * M_PI / 180.0; };
            const auto& sec = toml::find(cfg, "neck_takeover_control");
            if (sec.contains("yaw_range_deg"))
                yaw_range_rad_   = static_cast<float>(deg2rad(toml::find<double>(sec, "yaw_range_deg")));
            if (sec.contains("pitch_range_deg"))
                pitch_range_rad_ = static_cast<float>(deg2rad(toml::find<double>(sec, "pitch_range_deg")));
            if (sec.contains("max_step_deg"))
                max_step_rad_    = static_cast<float>(deg2rad(toml::find<double>(sec, "max_step_deg")));
            if (sec.contains("deadzone"))
                deadzone_        = std::clamp(static_cast<float>(toml::find<double>(sec, "deadzone")), 0.f, 1.f);
            // 数值参数合法性校验
            if (yaw_range_rad_ <= 0.0f) {
                std::cerr << "[neck_takeover_control] Warning: invalid yaw_range_deg<=0, reset to 45.0" << std::endl;
                yaw_range_rad_ = static_cast<float>(deg2rad(45.0));
            }
            if (pitch_range_rad_ <= 0.0f) {
                std::cerr << "[neck_takeover_control] Warning: invalid pitch_range_deg<=0, reset to 20.0" << std::endl;
                pitch_range_rad_ = static_cast<float>(deg2rad(20.0));
            }
            if (max_step_rad_ <= 0.0f) {
                std::cerr << "[neck_takeover_control] Warning: invalid max_step_deg<=0, reset to 0.03" << std::endl;
                max_step_rad_ = static_cast<float>(deg2rad(0.03));
            }
            const bool en = sec.contains("enable") ? toml::find<bool>(sec, "enable") : false;
            SetEnabled(en);
            const bool locked = sec.contains("mechanical_locked")
                                ? toml::find<bool>(sec, "mechanical_locked") : false;
            SetLocked(locked);
            std::cout << "[neck_takeover_control] enable=" << (en ? "true" : "false")
                      << ", mechanical_locked=" << (locked ? "true" : "false")
                      << ", yaw=±"   << (yaw_range_rad_   * 180.0 / M_PI) << "deg"
                      << ", pitch=±" << (pitch_range_rad_ * 180.0 / M_PI) << "deg"
                      << ", max_step=" << (max_step_rad_  * 180.0 / M_PI) << "deg/cycle"
                      << ", deadzone=" << deadzone_
                      << " (neck_dof_num=" << neck_dof_num << ")" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[neck_takeover_control] parse failed: " << e.what()
                      << ", using defaults" << std::endl;
        }
    }

    /// 在 SetJointCommand 入口调用，覆盖 input 末尾 neck_dof_num 个关节的
    /// 期望位置/速度/前馈力矩；kp/kd 保留 input 原值。
    /// neck_dof_num=2 → 覆盖 yaw+pitch；=1 → 仅覆盖 yaw；<=0 直接 short-circuit。
    /// yaw_actual_pos/pitch_actual_pos 为 neck 关节实际位置（来自 GetJointPosition），
    /// 仅用于启用后首帧锚定平滑起点；pitch 不存在时该参数被忽略。
    void Apply(Eigen::Matrix<float, Eigen::Dynamic, 5>& input,
               int dof_num, int neck_dof_num,
               float yaw_actual_pos, float pitch_actual_pos){
        if (neck_dof_num <= 0) return;
        if (input.rows() < dof_num) return;
        const int yaw_idx   = dof_num - neck_dof_num;
        const int pitch_idx = (neck_dof_num >= 2) ? (dof_num - 1) : -1;

        // 最高优先级：机械锁死 → neck 行整行清零（kp/pos/kd/vel/tau）
        if (locked_.load(std::memory_order_acquire)) {
            input.row(yaw_idx).setZero();
            if (pitch_idx >= 0) input.row(pitch_idx).setZero();
            // 锁死期间脖子可能被外力改变，解锁后下次 kp/kd 激活需重新锚定。
            prev_kp_kd_active_.store(false, std::memory_order_release);
            return;
        }

        if (!enabled_.load(std::memory_order_acquire)) {
            // 关闭期间上游控制脖子，filt 及边沿状态下次启用时会重置，不需要在这里特殊处理。
            return;
        }

        // 每帧检测上游是否在控制 neck（kp/kd 都 > 0）。仅在 inactive → active 的边沿
        // 首帧以关节实际位置锚定平滑起点，这样 idle/过渡段 → 控制段 反复切换都能重新锚定，
        // 不会出现“上次接管结束后被拉到 0 的 filt 残留状态”导致的甩零位问题。
        const bool yaw_active = (input(yaw_idx, 0) > 0.f) && (input(yaw_idx, 2) > 0.f);
        const bool pitch_active = (pitch_idx < 0) ||
                                  ((input(pitch_idx, 0) > 0.f) && (input(pitch_idx, 2) > 0.f));
        const bool kp_kd_active = yaw_active && pitch_active;
        if (!kp_kd_active) {
            // kp/kd 未激活（idle/过渡/锁姿态等）：本帧不覆盖，重置边沿状态使下次重新锚定。
            prev_kp_kd_active_.store(false, std::memory_order_release);
            return;
        }
        // 边沿触发：inactive → active 的首帧，用关节实际位置重新锚定。
        if (!prev_kp_kd_active_.exchange(true, std::memory_order_acq_rel)) {
            yaw_filt_.store(yaw_actual_pos, std::memory_order_release);
            pitch_filt_.store((pitch_idx >= 0) ? pitch_actual_pos : 0.f,
                              std::memory_order_release);
        }

        auto deadzone = [&](float v){
            return (std::fabs(v) < deadzone_) ? 0.f : v;
        };
        const float cy = std::clamp(deadzone(yaw_cmd_.load(std::memory_order_acquire)),   -1.f, 1.f);
        const float cp = std::clamp(deadzone(pitch_cmd_.load(std::memory_order_acquire)), -1.f, 1.f);
        const float yaw_target   = cy * yaw_range_rad_;
        const float pitch_target = cp * pitch_range_rad_;

        auto step_clamp = [&](float cur, float tgt){
            const float diff = tgt - cur;
            if (diff >  max_step_rad_) return cur + max_step_rad_;
            if (diff < -max_step_rad_) return cur - max_step_rad_;
            return tgt;
        };
        const float yaw_new   = step_clamp(yaw_filt_.load(std::memory_order_acquire),   yaw_target);
        const float pitch_new = step_clamp(pitch_filt_.load(std::memory_order_acquire), pitch_target);
        yaw_filt_.store(yaw_new,     std::memory_order_release);
        pitch_filt_.store(pitch_new, std::memory_order_release);

        input(yaw_idx, 1) = yaw_new;
        input(yaw_idx, 3) = 0.f;
        input(yaw_idx, 4) = 0.f;
        if (pitch_idx >= 0) {
            input(pitch_idx, 1) = pitch_new;
            input(pitch_idx, 3) = 0.f;
            input(pitch_idx, 4) = 0.f;
        }
    }

private:
    std::atomic<bool>  enabled_{false};
    std::atomic<bool>  locked_{false};   // 机械锁死开关
    std::atomic<bool>  prev_kp_kd_active_{false};  // 上一帧 kp/kd 是否激活，边沿检测重锚
    std::atomic<float> yaw_cmd_{0.f};
    std::atomic<float> pitch_cmd_{0.f};
    std::atomic<float> yaw_filt_{0.f};
    std::atomic<float> pitch_filt_{0.f};
    // 默认值：yaw ±45°、pitch ±20°、每帧 0.03°（1ms 控制周期 ≈ 30°/s）、死区 0.05
    float yaw_range_rad_   = 45.0f * static_cast<float>(M_PI) / 180.0f;
    float pitch_range_rad_ = 20.0f * static_cast<float>(M_PI) / 180.0f;
    float max_step_rad_    = 0.03f * static_cast<float>(M_PI) / 180.0f;
    float deadzone_        = 0.05f;
};

}  // namespace utils

#endif  // NECK_TAKEOVER_HPP_
