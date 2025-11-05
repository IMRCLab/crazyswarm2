
/**
 * @file crazyflie_ros.h
 * @brief ROS wrapper for Crazyflie C++ class
 */
#pragma once

#include <crazyflie_cpp/Crazyflie.h>
#include <tf2_ros/transform_broadcaster.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "crazyflie/crazyflie_logger.h"
#include "crazyflie_interfaces/msg/connection_statistics_array.hpp"
#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/msg/status.hpp"
#include "crazyflie_interfaces/msg/velocity_world.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"

namespace {}  // namespace

namespace crazyswarm2 {
// ROS wrapper for a single Crazyflie object
class CrazyflieROS {
public:
    using Arm = crazyflie_interfaces::srv::Arm;
    using GoTo = crazyflie_interfaces::srv::GoTo;
    using Land = crazyflie_interfaces::srv::Land;
    using NotifySetpointsStop = crazyflie_interfaces::srv::NotifySetpointsStop;
    using StartTrajectory = crazyflie_interfaces::srv::StartTrajectory;
    using Takeoff = crazyflie_interfaces::srv::Takeoff;
    using UploadTrajectory = crazyflie_interfaces::srv::UploadTrajectory;
    using Empty = std_srvs::srv::Empty;

    CrazyflieROS(const std::string &link_uri, const std::string &cf_type, const std::string &name, rclcpp::Node *node,
                 rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd,
                 rclcpp::CallbackGroup::SharedPtr callback_group_cf_srv, const CrazyflieBroadcaster *cfbc,
                 bool enable_parameters = true);

    void spin_once() { cf_.processAllPackets(); }

    std::string broadcastUri() const { return cf_.broadcastUri(); }

    uint8_t id() const { return cf_.address() & 0xFF; }

    const Crazyflie::ParamTocEntry *paramTocEntry(const std::string &group, const std::string &name) const;

    const std::string &name() const { return name_; }

    void change_parameter(const rclcpp::Parameter &p);

private:
    struct logPose {
        float x;
        float y;
        float z;
        int32_t quatCompressed;
    } __attribute__((packed));

    struct logScan {
        uint16_t front;
        uint16_t left;
        uint16_t back;
        uint16_t right;
    } __attribute__((packed));

    struct logOdom {
        int16_t x;
        int16_t y;
        int16_t z;
        int32_t quatCompressed;
        int16_t vx;
        int16_t vy;
        int16_t vz;
        // int16_t rateRoll;
        // int16_t ratePitch;
        // int16_t rateYaw;
    } __attribute__((packed));

    struct logStatus {
        // general status
        uint16_t supervisorInfo;  // supervisor.info
        // battery related
        // Note that using BQ-deck/Bolt one can actually have two batteries at the same time.
        // vbat refers to the battery directly connected to the CF board and might not reflect
        // the "external" battery on BQ/Bolt builds
        uint16_t vbatMV;  // pm.vbatMV
        uint8_t pmState;  // pm.state
        // radio related
        uint8_t rssi;      // radio.rssi
        uint16_t numRxBc;  // radio.numRxBc
        uint16_t numRxUc;  // radio.numRxUc
    } __attribute__((packed));

    void cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg);

    void cmd_velocity_world_changed(const crazyflie_interfaces::msg::VelocityWorld::SharedPtr msg);

    void cmd_position_changed(const crazyflie_interfaces::msg::Position::SharedPtr msg);

    void cmd_hover_changed(const crazyflie_interfaces::msg::Hover::SharedPtr msg);

    void cmd_vel_legacy_changed(const geometry_msgs::msg::Twist::SharedPtr msg);

    void on_console(const char *msg);

    void emergency(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response);

    void start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
                          std::shared_ptr<StartTrajectory::Response> response);

    void takeoff(const std::shared_ptr<Takeoff::Request> request, std::shared_ptr<Takeoff::Response> response);

    void land(const std::shared_ptr<Land::Request> request, std::shared_ptr<Land::Response> response);

    void go_to(const std::shared_ptr<GoTo::Request> request, std::shared_ptr<GoTo::Response> response);

    void upload_trajectory(const std::shared_ptr<UploadTrajectory::Request> request,
                           std::shared_ptr<UploadTrajectory::Response> response);

    void notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                               std::shared_ptr<NotifySetpointsStop::Response> response);

    void arm(const std::shared_ptr<Arm::Request> request, std::shared_ptr<Arm::Response> response);

    void on_logging_pose(uint32_t time_in_ms, const logPose *data);

    void on_logging_scan(uint32_t time_in_ms, const logScan *data);

    void on_logging_odom(uint32_t time_in_ms, const logOdom *data);

    void on_logging_status(uint32_t time_in_ms, const logStatus *data);

    void on_logging_custom(uint32_t time_in_ms, const std::vector<float> *values, void *userData);

    void on_link_statistics_timer();

    void on_latency(uint64_t latency_in_us);

    rclcpp::Logger logger_;
    CrazyflieLogger cf_logger_;

    Crazyflie cf_;
    std::string message_buffer_;
    std::string name_;

    rclcpp::Node *node_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    rclcpp::Service<Empty>::SharedPtr service_emergency_;
    rclcpp::Service<StartTrajectory>::SharedPtr service_start_trajectory_;
    rclcpp::Service<Takeoff>::SharedPtr service_takeoff_;
    rclcpp::Service<Land>::SharedPtr service_land_;
    rclcpp::Service<GoTo>::SharedPtr service_go_to_;
    rclcpp::Service<UploadTrajectory>::SharedPtr service_upload_trajectory_;
    rclcpp::Service<NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;
    rclcpp::Service<Arm>::SharedPtr service_arm_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_cmd_vel_legacy_;
    rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subscription_cmd_full_state_;
    rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr subscription_cmd_position_;
    rclcpp::Subscription<crazyflie_interfaces::msg::VelocityWorld>::SharedPtr subscription_cmd_velocity_world_;
    rclcpp::Subscription<crazyflie_interfaces::msg::Hover>::SharedPtr subscription_cmd_hover_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_robot_description_;

    // logging
    std::string reference_frame_;

    std::unique_ptr<LogBlock<logPose>> log_block_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_pose_;

    std::unique_ptr<LogBlock<logScan>> log_block_scan_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_scan_;

    std::unique_ptr<LogBlock<logOdom>> log_block_odom_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odom_;

    std::unique_ptr<LogBlock<logStatus>> log_block_status_;
    bool status_has_radio_stats_;
    rclcpp::Publisher<crazyflie_interfaces::msg::Status>::SharedPtr publisher_status_;
    uint16_t previous_numRxBc;
    uint16_t previous_numRxUc;
    bitcraze::crazyflieLinkCpp::Connection::Statistics previous_stats_unicast_;
    bitcraze::crazyflieLinkCpp::Connection::Statistics previous_stats_broadcast_;
    const CrazyflieBroadcaster *cfbc_;

    std::list<std::unique_ptr<LogBlockGeneric>> log_blocks_generic_;
    std::list<rclcpp::Publisher<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr> publishers_generic_;

    // multithreading
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_;
    rclcpp::TimerBase::SharedPtr spin_timer_;

    // link statistics
    rclcpp::TimerBase::SharedPtr link_statistics_timer_;
    std::chrono::time_point<std::chrono::steady_clock> last_on_latency_;
    uint16_t last_latency_in_ms_;
    float warning_freq_;
    float max_latency_;
    float min_ack_rate_;
    float min_unicast_receive_rate_;
    float min_broadcast_receive_rate_;
    bool publish_stats_;
    rclcpp::Publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>::SharedPtr publisher_connection_stats_;
};
}  // namespace crazyswarm2