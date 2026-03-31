/**
 * @file crazyflie_server.cpp
 * @brief defines CrazyflieServer class to manage multiple CrazyflieROS instances
 */

#include "crazyflie/crazyflie_server.h"

#include <crazyflie/crazyflie_ros.h>
#include <crazyflie_cpp/Crazyflie.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <vector>

#include "crazyflie_interfaces/msg/connection_statistics_array.hpp"
#include "crazyflie_interfaces/msg/full_state.hpp"
#include "crazyflie_interfaces/msg/hover.hpp"
#include "crazyflie_interfaces/msg/log_data_generic.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "crazyflie_interfaces/msg/status.hpp"
#include "crazyflie_interfaces/srv/arm.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/srv/start_trajectory.hpp"
#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/upload_trajectory.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "motion_capture_tracking_interfaces/msg/named_pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/empty.hpp"


namespace {
using std::placeholders::_1;
using std::placeholders::_2;

std::set<std::string> extract_names(const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides,
                                    const std::string &pattern) {
    std::set<std::string> result;
    for (const auto &i : parameter_overrides) {
        if (i.first.find(pattern) == 0) {
            size_t start = pattern.size() + 1;
            size_t end = i.first.find(".", start);
            result.insert(i.first.substr(start, end - start));
        }
    }
    return result;
}

#ifdef ROS_DISTRO_HUMBLE
auto get_service_qos() { return rmw_qos_profile_services_default; }
#else
auto get_service_qos() { return rclcpp::ServicesQoS(); }
#endif
}  // namespace

namespace crazyswarm2 {
CrazyflieServer::CrazyflieServer() : Node("crazyflie_server"), logger_(get_logger()) {
    // Create callback groups (each group can run in a separate thread)
    callback_group_mocap_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_mocap = rclcpp::SubscriptionOptions();
    sub_opt_mocap.callback_group = callback_group_mocap_;

    callback_group_all_cmd_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt_all_cmd = rclcpp::SubscriptionOptions();
    sub_opt_all_cmd.callback_group = callback_group_all_cmd_;

    callback_group_all_srv_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_cf_cmd_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    callback_group_cf_srv_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // declare global params
    this->declare_parameter("all.broadcasts.num_repeats", 15);
    this->declare_parameter("all.broadcasts.delay_between_repeats_ms", 1);
    this->declare_parameter("firmware_params.query_all_values_on_connect", false);

    broadcasts_num_repeats_ = this->get_parameter("all.broadcasts.num_repeats").get_parameter_value().get<int>();
    broadcasts_delay_between_repeats_ms_ =
        this->get_parameter("all.broadcasts.delay_between_repeats_ms").get_parameter_value().get<int>();
    mocap_enabled_ = false;

    this->declare_parameter("robot_description", "");

    // Warnings
    this->declare_parameter("warnings.frequency", 1.0);
    float freq = this->get_parameter("warnings.frequency").get_parameter_value().get<float>();
    if (freq >= 0.0) {
        watchdog_timer_ =
            this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / freq)),
                                    std::bind(&CrazyflieServer::on_watchdog_timer, this), callback_group_all_srv_);
    }
    this->declare_parameter("warnings.motion_capture.warning_if_rate_outside", std::vector<double>({80.0, 120.0}));
    auto rate_range = this->get_parameter("warnings.motion_capture.warning_if_rate_outside")
                          .get_parameter_value()
                          .get<std::vector<double>>();
    mocap_min_rate_ = rate_range[0];
    mocap_max_rate_ = rate_range[1];

    this->declare_parameter("warnings.communication.max_unicast_latency", 10.0);
    this->declare_parameter("warnings.communication.min_unicast_ack_rate", 0.9);
    this->declare_parameter("warnings.communication.min_unicast_receive_rate", 0.9);
    this->declare_parameter("warnings.communication.min_broadcast_receive_rate", 0.9);
    this->declare_parameter("warnings.communication.publish_stats", false);

    publish_stats_ = this->get_parameter("warnings.communication.publish_stats").get_parameter_value().get<bool>();
    if (publish_stats_) {
        publisher_connection_stats_ = this->create_publisher<crazyflie_interfaces::msg::ConnectionStatisticsArray>(
            "all/connection_statistics", 10);
    }

    // load crazyflies from params
    auto node_parameters_iface = this->get_node_parameters_interface();
    const std::map<std::string, rclcpp::ParameterValue> &parameter_overrides =
        node_parameters_iface->get_parameter_overrides();

    auto cf_names = extract_names(parameter_overrides, "robots");
    for (const auto &name : cf_names) {
        bool enabled = parameter_overrides.at("robots." + name + ".enabled").get<bool>();
        if (enabled) {
            // Lookup type
            std::string cf_type = parameter_overrides.at("robots." + name + ".type").get<std::string>();
            // Find the connection setting for the given type
            const auto con = parameter_overrides.find("robot_types." + cf_type + ".connection");
            std::string constr = "crazyflie";
            if (con != parameter_overrides.end()) {
                constr = con->second.get<std::string>();
            }
            // Find the mocap setting
            const auto mocap_en = parameter_overrides.find("robot_types." + cf_type + ".motion_capture.enabled");
            if (mocap_en != parameter_overrides.end()) {
                if (mocap_en->second.get<bool>()) {
                    mocap_enabled_ = true;
                }
            }

            // if it is a Crazyflie, try to connect
            if (constr == "crazyflie") {
                std::string uri = parameter_overrides.at("robots." + name + ".uri").get<std::string>();
                auto broadcastUri = Crazyflie::broadcastUriFromUnicastUri(uri);
                if (broadcaster_.count(broadcastUri) == 0) {
                    broadcaster_.emplace(broadcastUri, std::make_unique<CrazyflieBroadcaster>(broadcastUri));
                }

                crazyflies_.emplace(
                    name, std::make_unique<CrazyflieROS>(uri, cf_type, name, this, callback_group_cf_cmd_,
                                                         callback_group_cf_srv_, broadcaster_.at(broadcastUri).get()));

                update_name_to_id_map(name, crazyflies_[name]->id());
            } else if (constr == "none") {
                // we still might want to track this object, so update our map
                uint8_t id = parameter_overrides.at("robots." + name + ".id").get<uint8_t>();
                update_name_to_id_map(name, id);
            } else {
                RCLCPP_INFO(logger_, "[all] Unknown connection type %s", constr.c_str());
            }
        }
    }

    this->declare_parameter("poses_qos_deadline", 100.0f);
    double poses_qos_deadline = this->get_parameter("poses_qos_deadline").get_parameter_value().get<double>();

    rclcpp::SensorDataQoS sensor_data_qos;
    sensor_data_qos.keep_last(1);
    sensor_data_qos.deadline(rclcpp::Duration(0 /*s*/, 1e9 / poses_qos_deadline /*ns*/));
    sub_poses_ = this->create_subscription<NamedPoseArray>(
        "poses", sensor_data_qos, std::bind(&CrazyflieServer::posesChanged, this, _1), sub_opt_mocap);

    // support for all.params

    // Create a parameter subscriber that can be used to monitor parameter changes
    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
    cb_handle_ =
        param_subscriber_->add_parameter_event_callback(std::bind(&CrazyflieServer::on_parameter_event, this, _1));

    // topics for "all"
    subscription_cmd_full_state_ = this->create_subscription<crazyflie_interfaces::msg::FullState>(
        "all/cmd_full_state", rclcpp::SystemDefaultsQoS(),
        std::bind(&CrazyflieServer::cmd_full_state_changed, this, _1), sub_opt_all_cmd);

    // services for "all"
    service_start_trajectory_ = this->create_service<StartTrajectory>(
        "all/start_trajectory", std::bind(&CrazyflieServer::start_trajectory, this, _1, _2), get_service_qos(),
        callback_group_all_srv_);
    service_takeoff_ = this->create_service<Takeoff>("all/takeoff", std::bind(&CrazyflieServer::takeoff, this, _1, _2),
                                                     get_service_qos(), callback_group_all_srv_);
    service_land_ = this->create_service<Land>("all/land", std::bind(&CrazyflieServer::land, this, _1, _2),
                                               get_service_qos(), callback_group_all_srv_);
    service_go_to_ = this->create_service<GoTo>("all/go_to", std::bind(&CrazyflieServer::go_to, this, _1, _2),
                                                get_service_qos(), callback_group_all_srv_);
    service_notify_setpoints_stop_ = this->create_service<NotifySetpointsStop>(
        "all/notify_setpoints_stop", std::bind(&CrazyflieServer::notify_setpoints_stop, this, _1, _2),
        get_service_qos(), callback_group_all_srv_);
    service_arm_ = this->create_service<Arm>("all/arm", std::bind(&CrazyflieServer::arm, this, _1, _2),
                                             get_service_qos(), callback_group_all_srv_);

    // This is the last service to announce and can be used to check if the server is fully available
    service_emergency_ =
        this->create_service<Empty>("all/emergency", std::bind(&CrazyflieServer::emergency, this, _1, _2),
                                    get_service_qos(), callback_group_all_srv_);
}

void CrazyflieServer::emergency(const std::shared_ptr<Empty::Request> request,
                                std::shared_ptr<Empty::Response> response) {
    RCLCPP_INFO(logger_, "[all] emergency()");
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->emergencyStop();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
}

void CrazyflieServer::start_trajectory(const std::shared_ptr<StartTrajectory::Request> request,
                                       std::shared_ptr<StartTrajectory::Response> response) {
    RCLCPP_INFO(logger_, "[all] start_trajectory(id=%d, timescale=%f, reversed=%d, group_mask=%d)",
                request->trajectory_id, request->timescale, request->reversed, request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->startTrajectory(request->trajectory_id, request->timescale, request->reversed, request->group_mask);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
}

void CrazyflieServer::takeoff(const std::shared_ptr<Takeoff::Request> request,
                              std::shared_ptr<Takeoff::Response> response) {
    RCLCPP_INFO(logger_, "[all] takeoff(height=%f m, duration=%f s, group_mask=%d)", request->height,
                rclcpp::Duration(request->duration).seconds(), request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->takeoff(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
}

void CrazyflieServer::land(const std::shared_ptr<Land::Request> request, std::shared_ptr<Land::Response> response) {
    RCLCPP_INFO(logger_, "[all] land(height=%f m, duration=%f s, group_mask=%d)", request->height,
                rclcpp::Duration(request->duration).seconds(), request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->land(request->height, rclcpp::Duration(request->duration).seconds(), request->group_mask);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
}

void CrazyflieServer::go_to(const std::shared_ptr<GoTo::Request> request, std::shared_ptr<GoTo::Response> response) {
    RCLCPP_INFO(logger_, "[all] go_to(position=%f,%f,%f m, yaw=%f rad, duration=%f s, group_mask=%d)", request->goal.x,
                request->goal.y, request->goal.z, request->yaw, rclcpp::Duration(request->duration).seconds(),
                request->group_mask);
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->goTo(request->goal.x, request->goal.y, request->goal.z, request->yaw,
                       rclcpp::Duration(request->duration).seconds(), request->group_mask);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
}

void CrazyflieServer::notify_setpoints_stop(const std::shared_ptr<NotifySetpointsStop::Request> request,
                                            std::shared_ptr<NotifySetpointsStop::Response> response) {
    RCLCPP_INFO(logger_, "[all] notify_setpoints_stop(remain_valid_millisecs%d, group_mask=%d)",
                request->remain_valid_millisecs, request->group_mask);

    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->notifySetpointsStop(request->remain_valid_millisecs);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
}

void CrazyflieServer::arm(const std::shared_ptr<Arm::Request> request, std::shared_ptr<Arm::Response> response) {
    RCLCPP_INFO(logger_, "[all] arm(%d)", request->arm);

    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->sendArmingRequest(request->arm);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
}

void CrazyflieServer::cmd_full_state_changed(const crazyflie_interfaces::msg::FullState::SharedPtr msg) {
    float x = msg->pose.position.x;
    float y = msg->pose.position.y;
    float z = msg->pose.position.z;
    float vx = msg->twist.linear.x;
    float vy = msg->twist.linear.y;
    float vz = msg->twist.linear.z;
    float ax = msg->acc.x;
    float ay = msg->acc.y;
    float az = msg->acc.z;

    float qx = msg->pose.orientation.x;
    float qy = msg->pose.orientation.y;
    float qz = msg->pose.orientation.z;
    float qw = msg->pose.orientation.w;
    float rollRate = msg->twist.angular.x;
    float pitchRate = msg->twist.angular.y;
    float yawRate = msg->twist.angular.z;

    for (auto &bc : broadcaster_) {
        auto &cfbc = bc.second;
        cfbc->sendFullStateSetpoint(x, y, z, vx, vy, vz, ax, ay, az, qx, qy, qz, qw, rollRate, pitchRate, yawRate);
    }
}

void CrazyflieServer::posesChanged(const NamedPoseArray::SharedPtr msg) {
    mocap_data_received_timepoints_.emplace_back(std::chrono::steady_clock::now());

    // Here, we send all the poses to all CFs
    // In Crazyswarm1, we only sent the poses of the same group (i.e. channel)

    // split the message into parts that require position update and pose update
    std::vector<CrazyflieBroadcaster::externalPosition> data_position;
    std::vector<CrazyflieBroadcaster::externalPose> data_pose;

    for (const auto &pose : msg->poses) {
        const auto iter = name_to_id_.find(pose.name);
        if (iter != name_to_id_.end()) {
            uint8_t id = iter->second;
            if (isnan(pose.pose.orientation.w)) {
                data_position.push_back(
                    {id, (float)pose.pose.position.x, (float)pose.pose.position.y, (float)pose.pose.position.z});
            } else {
                data_pose.push_back({id, (float)pose.pose.position.x, (float)pose.pose.position.y,
                                     (float)pose.pose.position.z, (float)pose.pose.orientation.x,
                                     (float)pose.pose.orientation.y, (float)pose.pose.orientation.z,
                                     (float)pose.pose.orientation.w});
            }
        }
    }

    // send position only updates to the swarm
    if (data_position.size() > 0) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->sendExternalPositions(data_position);
        }
    }

    // send pose only updates to the swarm
    if (data_pose.size() > 0) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->sendExternalPoses(data_pose);
        }
    }
}

void CrazyflieServer::on_parameter_event(const rcl_interfaces::msg::ParameterEvent &event) {
    if (event.node == "/crazyflie_server") {
        auto params = param_subscriber_->get_parameters_from_event(event);
        for (auto &p : params) {
            size_t params_pos = p.get_name().find(".params.");
            if (params_pos == std::string::npos) {
                continue;
            }
            std::string cfname(p.get_name().begin(), p.get_name().begin() + params_pos);
            size_t prefixsize = params_pos + 8;
            if (cfname == "all") {
                size_t pos = p.get_name().find(".", prefixsize);
                std::string group(p.get_name().begin() + prefixsize, p.get_name().begin() + pos);
                std::string name(p.get_name().begin() + pos + 1, p.get_name().end());

                RCLCPP_INFO(logger_, "[all] Update parameter \"%s.%s\" to %s", group.c_str(), name.c_str(),
                            p.value_to_string().c_str());

                Crazyflie::ParamType paramType;
                for (auto &cf : crazyflies_) {
                    const auto entry = cf.second->paramTocEntry(group, name);
                    if (entry) {
                        switch (entry->type) {
                            case Crazyflie::ParamTypeUint8:
                                broadcast_set_param<uint8_t>(group, name, p.as_int());
                                break;
                            case Crazyflie::ParamTypeInt8:
                                broadcast_set_param<int8_t>(group, name, p.as_int());
                                break;
                            case Crazyflie::ParamTypeUint16:
                                broadcast_set_param<uint16_t>(group, name, p.as_int());
                                break;
                            case Crazyflie::ParamTypeInt16:
                                broadcast_set_param<int16_t>(group, name, p.as_int());
                                break;
                            case Crazyflie::ParamTypeUint32:
                                broadcast_set_param<uint32_t>(group, name, p.as_int());
                                break;
                            case Crazyflie::ParamTypeInt32:
                                broadcast_set_param<int32_t>(group, name, p.as_int());
                                break;
                            case Crazyflie::ParamTypeFloat:
                                if (p.get_type() == rclcpp::PARAMETER_INTEGER) {
                                    broadcast_set_param<float>(group, name, (float)p.as_int());
                                } else {
                                    broadcast_set_param<float>(group, name, p.as_double());
                                }
                                break;
                        }
                        break;
                    }
                }
            } else {
                auto iter = crazyflies_.find(cfname);
                if (iter != crazyflies_.end()) {
                    iter->second->change_parameter(p);
                }
            }
        }
    }
}

void CrazyflieServer::on_watchdog_timer() {
    auto now = std::chrono::steady_clock::now();

    // motion capture
    // a) check if the rate was within specified bounds
    if (mocap_data_received_timepoints_.size() >= 2) {
        double mean_rate = 0;
        double min_rate = std::numeric_limits<double>::max();
        double max_rate = 0;
        int num_rates_wrong = 0;
        for (size_t i = 0; i < mocap_data_received_timepoints_.size() - 1; ++i) {
            std::chrono::duration<double> diff =
                mocap_data_received_timepoints_[i + 1] - mocap_data_received_timepoints_[i];
            double rate = 1.0 / diff.count();
            mean_rate += rate;
            min_rate = std::min(min_rate, rate);
            max_rate = std::max(max_rate, rate);
            if (rate <= mocap_min_rate_ || rate >= mocap_max_rate_) {
                num_rates_wrong++;
            }
        }
        mean_rate /= (mocap_data_received_timepoints_.size() - 1);

        if (num_rates_wrong > 0) {
            RCLCPP_WARN(logger_, "[all] Motion capture rate off (#: %d, Avg: %.1f, Min: %.1f, Max: %.1f)",
                        num_rates_wrong, mean_rate, min_rate, max_rate);
        }
    } else if (mocap_enabled_) {
        // b) warn if no data was received
        RCLCPP_WARN(logger_, "[all] Motion capture did not receive data!");
    }

    mocap_data_received_timepoints_.clear();

    if (publish_stats_) {
        crazyflie_interfaces::msg::ConnectionStatisticsArray msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "world";  // this is across broadcasters, which is not directly associated with single
                                        // CFs; hence keep "world" here
        msg.stats.resize(broadcaster_.size());

        size_t i = 0;
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;

            auto stats = cfbc->connectionStatsDelta();

            msg.stats[i].uri = cfbc->uri();
            msg.stats[i].sent_count = stats.sent_count;
            msg.stats[i].sent_ping_count = stats.sent_ping_count;
            msg.stats[i].receive_count = stats.receive_count;
            msg.stats[i].enqueued_count = stats.enqueued_count;
            msg.stats[i].ack_count = stats.ack_count;
            ++i;
        }
        publisher_connection_stats_->publish(msg);
    }
}

template <class T>
void CrazyflieServer::broadcast_set_param(const std::string &group, const std::string &name, const T &value) {
    for (int i = 0; i < broadcasts_num_repeats_; ++i) {
        for (auto &bc : broadcaster_) {
            auto &cfbc = bc.second;
            cfbc->setParam<T>(group.c_str(), name.c_str(), value);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(broadcasts_delay_between_repeats_ms_));
    }
}

void CrazyflieServer::update_name_to_id_map(const std::string &name, uint8_t id) {
    const auto iter = name_to_id_.find(name);
    if (iter != name_to_id_.end()) {
        RCLCPP_WARN(logger_, "[all] At least two objects with the same id (%d, %s, %s)", id, name.c_str(),
                    iter->first.c_str());
    } else {
        name_to_id_.insert(std::make_pair(name, id));
    }
}
}  // namespace crazyswarm2