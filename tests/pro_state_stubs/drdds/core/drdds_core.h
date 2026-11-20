#pragma once
// Minimal DDS UInt64 channel used by the offline state-machine tests.
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace std_msgs {
namespace msg {
struct UInt64 {
    uint64_t value = 0;
    uint64_t data() const { return value; }
    void data(uint64_t new_value) { value = new_value; }
};
}  // namespace msg
}  // namespace std_msgs

class ChannelUInt64 {
public:
    using Callback = std::function<void(const std_msgs::msg::UInt64*)>;

    ChannelUInt64(Callback callback, std::string topic, int)
        : topic_(std::move(topic)) {
        Registry()[this] = {topic_, std::move(callback)};
    }

    ~ChannelUInt64() {
        Registry().erase(this);
    }

private:
    struct Registration {
        std::string topic;
        Callback callback;
    };

    static std::unordered_map<const ChannelUInt64*, Registration>& Registry() {
        static std::unordered_map<const ChannelUInt64*, Registration> registry;
        return registry;
    }

    static void Publish(const std::string& topic, uint64_t value) {
        std::vector<Callback> callbacks;
        for (const auto& [channel, registration] : Registry()) {
            if (registration.topic == topic) callbacks.push_back(registration.callback);
        }
        std_msgs::msg::UInt64 message;
        message.data(value);
        for (const auto& callback : callbacks) callback(&message);
    }

    std::string topic_;

    friend void PublishTestUInt64(const std::string&, uint64_t);
};

inline void PublishTestUInt64(const std::string& topic, uint64_t value) {
    ChannelUInt64::Publish(topic, value);
}
