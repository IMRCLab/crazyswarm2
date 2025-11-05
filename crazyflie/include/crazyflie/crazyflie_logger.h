/**
 * @file crazyflie_logger.h
 * @brief Helper class to convert crazyflie_cpp logging messages to ROS logging messages
 */

#pragma once

#include <crazyflie_cpp/Crazyflie.h>

#include <rclcpp/rclcpp.hpp>

// Note on logging: we use a single logger with string prefixes
// A better way would be to use named child loggers, but these do not
// report to /rosout in humble, see https://github.com/ros2/rclpy/issues/1131
// Once we do not support humble anymore, consider switching to child loggers

// Helper class to convert crazyflie_cpp logging messages to ROS logging messages
class CrazyflieLogger : public Logger {
public:
    CrazyflieLogger(rclcpp::Logger logger, const std::string &prefix) : Logger(), logger_(logger), prefix_(prefix) {}

    virtual ~CrazyflieLogger() {}

    virtual void info(const std::string &msg) { RCLCPP_INFO(logger_, "%s %s", prefix_.c_str(), msg.c_str()); }

    virtual void warning(const std::string &msg) { RCLCPP_WARN(logger_, "%s %s", prefix_.c_str(), msg.c_str()); }

    virtual void error(const std::string &msg) { RCLCPP_ERROR(logger_, "%s %s", prefix_.c_str(), msg.c_str()); }

private:
    rclcpp::Logger logger_;
    std::string prefix_;
};