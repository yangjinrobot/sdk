// =====================================================================
// runtime_params.hpp —— 机器人运行期参数实时快照模块
// ---------------------------------------------------------------------
// 设计目标：
//   1. 各模块（SafeController / JointProtection / TerrainPolicy 等）
//      在加载完毕、参数生效后调用 Set* 接口，将"实际生效值"写入本模块；
//   2. 模块内部周期性把缓存写入 conf/runtime_params.toml，外部即可通过
//      `tail -F conf/runtime_params.toml` 或文本查看器实时查看；
//   3. 读写均加锁，可在任意线程安全调用。
//
// 使用示例：
//   auto& rp = RuntimeParams::Instance();
//   rp.Start("conf/runtime_params.toml", 1000);   // 1Hz 自动回写
//   rp.SetBool ("joint_protection", "enabled", true);
//   rp.SetInt  ("joint_protection", "l2_threshold", 10);
//   rp.SetDouble("joint_protection", "damping_kd", 10.0);
//   rp.SetString("robot", "model", "CR1_PRO");
//   rp.SetDoubleArray("terrain_policy", "body_com", {0.0, 0.0, 0.0});
//   ...
//   rp.Stop();    // 程序退出前调用
// =====================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <variant>
#include <vector>

class RuntimeParams {
public:
    using ValueT = std::variant<bool, long long, double, std::string,
                                std::vector<double>>;

    static RuntimeParams& Instance() {
        static RuntimeParams inst;
        return inst;
    }

    // ---------------- 写入接口（供各模块调用） ----------------
    // 返回值：true 写入成功；false 表示 section/key 为空已被拒绝（已 stderr 告警）
    bool SetBool(const std::string& section, const std::string& key, bool v) {
        return SetValue(section, key, ValueT(v));
    }
    bool SetInt(const std::string& section, const std::string& key, long long v) {
        return SetValue(section, key, ValueT(v));
    }
    bool SetDouble(const std::string& section, const std::string& key, double v) {
        return SetValue(section, key, ValueT(v));
    }
    bool SetString(const std::string& section, const std::string& key,
                   const std::string& v) {
        return SetValue(section, key, ValueT(v));
    }
    bool SetDoubleArray(const std::string& section, const std::string& key,
                        const std::vector<double>& v) {
        return SetValue(section, key, ValueT(v));
    }

    // 删除某个键；section/key 为空时返回 false 并告警
    bool Remove(const std::string& section, const std::string& key) {
        if (section.empty() || key.empty()) {
            std::fprintf(stderr,
                         "[RuntimeParams] Remove rejected: empty section('%s') or key('%s')\n",
                         section.c_str(), key.c_str());
            return false;
        }
        std::lock_guard<std::mutex> lk(mu_);
        auto it = data_.find(section);
        if (it != data_.end()) {
            it->second.erase(key);
            dirty_ = true;
        }
        return true;
    }

    // 立即写入文件（同步），不依赖后台线程；返回 true 表示成功
    bool Flush() {
        std::string path_copy;
        {
            std::lock_guard<std::mutex> lk(mu_);
            path_copy = file_path_;
        }
        return WriteToFile(path_copy);
    }

    // 指定路径立即写入
    bool WriteToFile(const std::string& path) {
        if (path.empty()) return false;
        std::lock_guard<std::mutex> lk(mu_);
        return WriteLocked(path);
    }

    // 启动后台周期回写线程
    //   path       : 输出文件路径
    //   period_ms  : 回写周期（毫秒），<=0 时按 1000ms 处理
    void Start(const std::string& path, int period_ms = 1000) {
        Stop();
        {
            std::lock_guard<std::mutex> lk(mu_);
            file_path_ = path;
            period_ms_ = period_ms > 0 ? period_ms : 1000;
            running_   = true;
        }
        worker_ = std::thread(&RuntimeParams::WorkerLoop, this);
    }

    // 自动探测部署/开发路径并启动后台回写
    void Start(int period_ms = 1000) {
        static const char* kCandidates[] = {
            "/var/opt/robot/conf/motion_master/runtime_params.toml",
            "conf/runtime_params.toml",
        };
        std::string path = "conf/runtime_params.toml";
        for (const char* c : kCandidates) {
            std::filesystem::path p(c);
            if (std::filesystem::exists(p.parent_path())) { path = c; break; }
        }
        Start(path, period_ms);
    }

    // 停止后台线程并 flush 一次
    void Stop() {
        {
            std::lock_guard<std::mutex> lk(mu_);
            if (!running_) return;
            running_ = false;
        }
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
        // 退出前确保最后状态被持久化（Flush 内部已加锁拷贝 file_path_，
        // 并在 WriteToFile 中对空路径直接返回 false）
        if (!GetFilePath().empty()) Flush();
    }

    // 当前文件路径（调试/外部工具）
    std::string GetFilePath() {
        std::lock_guard<std::mutex> lk(mu_);
        return file_path_;
    }

    ~RuntimeParams() { Stop(); }

private:
    RuntimeParams() = default;
    RuntimeParams(const RuntimeParams&) = delete;
    RuntimeParams& operator=(const RuntimeParams&) = delete;

    bool SetValue(const std::string& section, const std::string& key,
                  ValueT v) {
        if (section.empty() || key.empty()) {
            std::fprintf(stderr,
                         "[RuntimeParams] SetValue rejected: empty section('%s') or key('%s')\n",
                         section.c_str(), key.c_str());
            return false;
        }
        std::lock_guard<std::mutex> lk(mu_);
        data_[section][key] = std::move(v);
        dirty_              = true;
        return true;
    }

    void WorkerLoop() {
        std::unique_lock<std::mutex> lk(mu_);
        while (running_) {
            cv_.wait_for(lk, std::chrono::milliseconds(period_ms_),
                         [this] { return !running_; });
            if (!running_) break;
            if (dirty_ && !file_path_.empty()) {
                WriteLocked(file_path_);
            }
        }
    }

    // 调用者必须持有 mu_
    bool WriteLocked(const std::string& path) {
        // 写到临时文件再 rename，避免中间态被读取到半行
        std::string tmp = path + ".tmp";
        FILE* fp        = std::fopen(tmp.c_str(), "w");
        if (!fp) return false;

        std::fprintf(fp,
                     "# =============================================\n"
                     "# robot runtime parameters (auto-generated)\n"
                     "# DO NOT edit manually, will be overwritten.\n"
                     "# =============================================\n");
        std::fprintf(fp, "last_update = \"%s\"\n\n", NowTimeStr().c_str());

        for (const auto& sec : data_) {
            std::fprintf(fp, "[%s]\n", sec.first.c_str());
            for (const auto& kv : sec.second) {
                std::fprintf(fp, "%s = %s\n", kv.first.c_str(),
                             FormatValue(kv.second).c_str());
            }
            std::fprintf(fp, "\n");
        }
        std::fflush(fp);
        std::fclose(fp);

        // 原子替换
        if (std::rename(tmp.c_str(), path.c_str()) != 0) {
            std::remove(tmp.c_str());
            return false;
        }
        dirty_ = false;
        return true;
    }

    static std::string FormatValue(const ValueT& v) {
        std::ostringstream os;
        std::visit(
            [&os](auto&& arg) {
                using T = std::decay_t<decltype(arg)>;
                if constexpr (std::is_same_v<T, bool>) {
                    os << (arg ? "true" : "false");
                } else if constexpr (std::is_same_v<T, long long>) {
                    os << arg;
                } else if constexpr (std::is_same_v<T, double>) {
                    // 6 位小数，避免长尾
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "%.6g", arg);
                    os << buf;
                } else if constexpr (std::is_same_v<T, std::string>) {
                    os << "\"" << EscapeStr(arg) << "\"";
                } else if constexpr (std::is_same_v<T, std::vector<double>>) {
                    os << "[";
                    for (size_t i = 0; i < arg.size(); ++i) {
                        char buf[64];
                        std::snprintf(buf, sizeof(buf), "%.6g", arg[i]);
                        os << buf;
                        if (i + 1 < arg.size()) os << ", ";
                    }
                    os << "]";
                }
            },
            v);
        return os.str();
    }

    static std::string EscapeStr(const std::string& s) {
        std::string out;
        out.reserve(s.size());
        for (char c : s) {
            if (c == '"' || c == '\\') out.push_back('\\');
            out.push_back(c);
        }
        return out;
    }

    static std::string NowTimeStr() {
        std::time_t t = std::time(nullptr);
        std::tm tm_local{};
#ifdef _WIN32
        localtime_s(&tm_local, &t);
#else
        localtime_r(&t, &tm_local);
#endif
        char buf[32];
        std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_local);
        return std::string(buf);
    }

    // section -> (key -> value)
    std::map<std::string, std::map<std::string, ValueT>> data_;
    std::mutex                                           mu_;
    std::condition_variable                              cv_;
    std::thread                                          worker_;
    std::string                                          file_path_;
    int                                                  period_ms_{1000};
    bool                                                 running_{false};
    bool                                                 dirty_{false};
};
