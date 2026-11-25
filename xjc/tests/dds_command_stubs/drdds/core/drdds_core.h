#pragma once
#ifndef MOTION_MASTER_DDS_TESTS
#error "The command DDS adapter must never be used in the hardware build"
#endif
#include <algorithm>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace std_msgs::msg {
struct String {
    std::string value;
    const std::string& data() const { return value; }
};
}
namespace drdds::msg {
struct MotionStateValue {
    uint32_t value;
    uint32_t state() const { return value; }
};
struct MotionState {
    MotionStateValue value;
    const MotionStateValue& data() const { return value; }
};
struct GaitValue {
    uint32_t value;
    uint32_t gait() const { return value; }
};
struct Gait {
    GaitValue value;
    const GaitValue& data() const { return value; }
};
struct SteerValue {
    float forward = 0, side = 0, turn = 0, tilt = 0;
    float x() const { return forward; }
    float y() const { return side; }
    float yaw() const { return turn; }
    float pitch() const { return tilt; }
};
struct Steer {
    SteerValue value;
    const SteerValue& data() const { return value; }
};
}

// Exercise the real subscription callbacks without creating a DDS participant.
template <typename Message>
class TestCommandChannel {
    inline static std::mutex mutex_;
    inline static std::vector<TestCommandChannel*> readers_;
    std::function<void(const Message*)> callback_;
    std::string topic_;
    int domain_;
public:
    TestCommandChannel(std::function<void(const Message*)> callback, const std::string& topic, int domain)
        : callback_(std::move(callback)), topic_(topic), domain_(domain) {
        std::lock_guard<std::mutex> lock(mutex_);
        readers_.push_back(this);
    }
    ~TestCommandChannel() {
        std::lock_guard<std::mutex> lock(mutex_);
        readers_.erase(std::remove(readers_.begin(), readers_.end(), this), readers_.end());
    }
    static int Publish(const Message& message, const std::string& topic, int domain = 0) {
        std::lock_guard<std::mutex> lock(mutex_);
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
using ChannelString = TestCommandChannel<std_msgs::msg::String>;
using ChannelSteer = TestCommandChannel<drdds::msg::Steer>;
using ChannelMotionState = TestCommandChannel<drdds::msg::MotionState>;
using ChannelGait = TestCommandChannel<drdds::msg::Gait>;
