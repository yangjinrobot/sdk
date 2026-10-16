#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <stdio.h>
#include <chrono>

class PointCloudSubscriber : public rclcpp::Node {
public:
  PointCloudSubscriber() : Node("pointcloud_subscriber") {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/LIDAR/POINTS", 10,
        std::bind(&PointCloudSubscriber::topic_callback, this,
                  std::placeholders::_1));
  }

private:
  void
  topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
    auto current_time = std::chrono::system_clock::now();

    auto msg_time_point = std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>(
        std::chrono::seconds(msg->header.stamp.sec) + std::chrono::nanoseconds(msg->header.stamp.nanosec)
    );
    
    double delay = std::chrono::duration<double>(current_time - msg_time_point).count();
    
    RCLCPP_INFO(this->get_logger(), "Received message with delay: %.6f seconds", delay);
    RCLCPP_INFO(this->get_logger(), "Width: %u, Height: %u", msg->width, msg->height);
    RCLCPP_INFO(this->get_logger(), "Point step: %u, Row step: %u", msg->point_step, msg->row_step);
    
    for (const auto &field : msg->fields) {
      RCLCPP_INFO(this->get_logger(), " - Name: %s, Offset: %u, Datatype: %u, Count: %u",
             field.name.c_str(), field.offset, field.datatype, field.count);
    }
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudSubscriber>());
  rclcpp::shutdown();
  return 0;
}