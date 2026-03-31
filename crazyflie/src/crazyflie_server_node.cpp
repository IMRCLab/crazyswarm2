/**
 * @file crazyflie_server_node.cpp
 * @brief main file for crazyflie_server node
 */

#include <rclcpp/rclcpp.hpp>
#include "crazyflie/crazyflie_server.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<crazyswarm2::CrazyflieServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}