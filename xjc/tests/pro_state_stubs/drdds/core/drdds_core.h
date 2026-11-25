#pragma once
#ifndef MOTION_MASTER_STATE_TESTS
#error "The test DDS adapter must never be used in the hardware build"
#endif
#include <algorithm>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace std_msgs::msg {
class UInt64 {
    uint64_t value_;
public:
    explicit UInt64(uint64_t value) : value_(value) {}
    uint64_t data() const { return value_; }
};
}

// Deliver test messages through the same callback/topic/domain contract as DDS.
class ChannelUInt64 {
    inline static std::mutex mutex_;
    inline static std::vector<ChannelUInt64*> readers_;
    std::function<void(const std_msgs::msg::UInt64*)> callback_;
    std::string topic_;
    int domain_;
public:
    ChannelUInt64(std::function<void(const std_msgs::msg::UInt64*)> callback,
                  const std::string& topic, int domain)
        : callback_(std::move(callback)), topic_(topic), domain_(domain) {
        std::lock_guard<std::mutex> lock(mutex_);
        readers_.push_back(this);
    }
    ~ChannelUInt64() {
        std::lock_guard<std::mutex> lock(mutex_);
        readers_.erase(std::remove(readers_.begin(), readers_.end(), this), readers_.end());
    }
    static int Publish(uint64_t mask, const std::string& topic = "/JOINTS_MARK_ZERO", int domain = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        const std_msgs::msg::UInt64 message(mask);
        int delivered = 0;
        for (const auto reader : readers_) {
            if (reader->topic_ == topic && reader->domain_ == domain) {
                reader->callback_(&message);
                ++delivered;
            }
        }
        return delivered;
    }
};
