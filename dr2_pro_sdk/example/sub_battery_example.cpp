#include "drdds/msg/battery_data.hpp"
#include "rclcpp/rclcpp.hpp"

class BatterySubscriber : public rclcpp::Node {
public:
    BatterySubscriber() : Node("sub_battery_example") {
        subscription_ = create_subscription<drdds::msg::BatteryData>(
            "/BATTERY_DATA", 10, std::bind(&BatterySubscriber::Callback, this, std::placeholders::_1));
    }

private:
    void Callback(const drdds::msg::BatteryData::SharedPtr msg) const {
        if (msg->data.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty BATTERY_DATA");
            return;
        }

        const auto& battery = msg->data.front();
        RCLCPP_INFO(get_logger(), "battery_level=%u voltage=%u current=%d protected_state=%u",
                    static_cast<unsigned>(battery.battery_level), static_cast<unsigned>(battery.voltage),
                    static_cast<int>(battery.current), static_cast<unsigned>(battery.protected_state));
    }

    rclcpp::Subscription<drdds::msg::BatteryData>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BatterySubscriber>());
    rclcpp::shutdown();
    return 0;
}
