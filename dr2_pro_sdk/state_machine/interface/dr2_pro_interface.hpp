#pragma once

#include "basic_function.hpp"
#include "robot_interface.hpp"

/**
 * @class DR2ProInterface
 * @brief Hardware interface implementation for M20 series robots
 *
 * @details Specializes DDS interface for M20-specific configurations:
 * - Custom joint offset initialization
 * - Joint position calibration sequence
 * - Wheel joint special handling
 */
class DR2ProInterface : public interface::RobotInterface {
public:
    explicit DR2ProInterface(rclcpp::Node::SharedPtr node, const std::string& robot_name = "DR2PRO") 
        : interface::RobotInterface(robot_name, 31, 12, 14, 3, 2, node) {
    }
    ~DR2ProInterface() {};
};
