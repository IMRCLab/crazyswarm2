/**
 * @file crazyflie_server.h
 * @brief defines CrazyflieServer class to manage multiple CrazyflieROS instances
 */
#pragma once

#include <crazyflie_cpp/Crazyflie.h>

#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "crazyflie/crazyflie_ros.h"
#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "std_srvs/srv/empty.hpp"

namespace crazyswarm2 {
class CrazyflieServer : public rclcpp::Node {
public:
    using NamedPoseArray = motion_capture_tracking_interfaces::msg::NamedPoseArray;
    using Arm = crazyflie_interfaces::srv::Arm;
    using GoTo = crazyflie_interfaces::srv::GoTo;
    using Land = crazyflie_interfaces::srv::Land;
    using NotifySetpointsStop = crazyflie_interfaces::srv::NotifySetpointsStop;
    using StartTrajectory = crazyflie_interfaces::srv::StartTrajectory;
    using Takeoff = crazyflie_interfaces::srv::Takeoff;
    using UploadTrajectory = crazyflie_interfaces::srv::UploadTrajectory;
    using Empty = std_srvs::srv::Empty;
    CrazyflieServer();

private:
    void emergency(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response);

    void start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
                          std::shared_ptr<StartTrajectory::Response> response);

    void takeoff(const std::shared_ptr<Takeoff::Request> request, std::shared_ptr<Takeoff::Response> response);

    void land(const std::shared_ptr<Land::Request> request, std::shared_ptr<Land::Response> response);

    void go_to(const std::shared_ptr<GoTo::Request> request, std::shared_ptr<GoTo::Response> response);

    void notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                               std::shared_ptr<NotifySetpointsStop::Response> response);

    void arm(const std::shared_ptr<Arm::Request> request, std::shared_ptr<Arm::Response> response);

    void cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg);

    void posesChanged(const NamedPoseArray::SharedPtr msg);

    void on_parameter_event(const rcl_interfaces::msg::ParameterEvent &event);

    void on_watchdog_timer();

    template <class T>
    void broadcast_set_param(const std::string &group, const std::string &name, const T &value);

    void update_name_to_id_map(const std::string &name, uint8_t id);

    rclcpp::Logger logger_;

    // subscribers
    rclcpp::Subscription<crazyflie_interfaces::msg::FullState>::SharedPtr subscription_cmd_full_state_;
    rclcpp::Subscription<NamedPoseArray>::SharedPtr sub_poses_;

    // services
    rclcpp::Service<Empty>::SharedPtr service_emergency_;
    rclcpp::Service<StartTrajectory>::SharedPtr service_start_trajectory_;
    rclcpp::Service<Takeoff>::SharedPtr service_takeoff_;
    rclcpp::Service<Land>::SharedPtr service_land_;
    rclcpp::Service<GoTo>::SharedPtr service_go_to_;
    rclcpp::Service<NotifySetpointsStop>::SharedPtr service_notify_setpoints_stop_;
    rclcpp::Service<Arm>::SharedPtr service_arm_;

    std::map<std::string, std::unique_ptr<CrazyflieROS>> crazyflies_;

    // broadcastUri -> broadcast object
    std::map<std::string, std::unique_ptr<CrazyflieBroadcaster>> broadcaster_;

    // maps CF name -> CF id
    std::map<std::string, uint8_t> name_to_id_;

    // global params
    int broadcasts_num_repeats_;
    int broadcasts_delay_between_repeats_ms_;

    // parameter updates
    std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
    std::shared_ptr<rclcpp::ParameterEventCallbackHandle> cb_handle_;

    // sanity checks
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    bool mocap_enabled_;
    float mocap_min_rate_;
    float mocap_max_rate_;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> mocap_data_received_timepoints_;
    bool publish_stats_;
    rclcpp::Publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>::SharedPtr publisher_connection_stats_;

    // multithreading
    rclcpp::CallbackGroup::SharedPtr callback_group_mocap_;
    rclcpp::CallbackGroup::SharedPtr callback_group_all_cmd_;
    rclcpp::CallbackGroup::SharedPtr callback_group_all_srv_;
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_cmd_;
    rclcpp::CallbackGroup::SharedPtr callback_group_cf_srv_;
};
}  // namespace crazyswarm2