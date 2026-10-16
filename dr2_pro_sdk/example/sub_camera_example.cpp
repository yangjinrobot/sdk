#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <iomanip>
#include <sstream>

class CameraSubscriber : public rclcpp::Node
{
public:
    CameraSubscriber()
        : Node("camera_subscriber")
    {
        // 注意：深度相机需在10.21.33.106上订阅
        // 订阅相机1深度图像
        camera1_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/camera1/depth/image_rect_raw", 10,
            std::bind(&CameraSubscriber::camera1DepthCallback, this, std::placeholders::_1));

        // 订阅相机2深度图像
        camera2_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera2/camera2/depth/image_rect_raw", 10,
            std::bind(&CameraSubscriber::camera2DepthCallback, this, std::placeholders::_1));

        // 订阅相机3深度图像
        camera3_depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera3/camera3/depth/image_rect_raw", 10,
            std::bind(&CameraSubscriber::camera3DepthCallback, this, std::placeholders::_1));

        // 订阅相机1RGB图像
        camera1_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera1/camera1/color/image_raw", 10,
            std::bind(&CameraSubscriber::camera1ColorCallback, this, std::placeholders::_1));

        // 订阅相机2RGB图像
        camera2_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera2/camera2/color/image_raw", 10,
            std::bind(&CameraSubscriber::camera2ColorCallback, this, std::placeholders::_1));

        // 订阅相机3RGB图像
        camera3_color_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera3/camera3/color/image_raw", 10,
            std::bind(&CameraSubscriber::camera3ColorCallback, this, std::placeholders::_1));

        // 广角相机取流地址
        //rtsp://10.21.33.103:8554/video1
    }

private:
    std::string formatTimestamp(const rclcpp::Time& stamp)
    {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(9);
        ss << stamp.seconds();
        return ss.str();
    }

    void camera1DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time stamp = msg->header.stamp;
        RCLCPP_INFO(this->get_logger(), "[Camera1][Depth]   timestamp: %s", formatTimestamp(stamp).c_str());
    }

    void camera2DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time stamp = msg->header.stamp;
        RCLCPP_INFO(this->get_logger(), "[Camera2][Depth]   timestamp: %s", formatTimestamp(stamp).c_str());
    }

    void camera3DepthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time stamp = msg->header.stamp;
        RCLCPP_INFO(this->get_logger(), "[Camera3][Depth]   timestamp: %s", formatTimestamp(stamp).c_str());
    }

    void camera1ColorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time stamp = msg->header.stamp;
        RCLCPP_INFO(this->get_logger(), "[Camera1][RGB]   timestamp: %s", formatTimestamp(stamp).c_str());
    }

    void camera2ColorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time stamp = msg->header.stamp;
        RCLCPP_INFO(this->get_logger(), "[Camera2][RGB]   timestamp: %s", formatTimestamp(stamp).c_str());
    }

    void camera3ColorCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        rclcpp::Time stamp = msg->header.stamp;
        RCLCPP_INFO(this->get_logger(), "[Camera3][RGB]   timestamp: %s", formatTimestamp(stamp).c_str());
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera1_depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera2_depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera3_depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera1_color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera2_color_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera3_color_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
