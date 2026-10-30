#include "crazyflie_rviz/status_display.hpp"

#include <iomanip>
#include <memory>
#include <sstream>

#include "pluginlib/class_list_macros.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace crazyflie_rviz
{

StatusDisplay::StatusDisplay()
: marker_common_(std::make_unique<rviz_default_plugins::displays::MarkerCommon>(this))
{
}

void StatusDisplay::onInitialize()
{
  RTDClass::onInitialize();
  marker_common_->initialize(context_, scene_node_);
}

void StatusDisplay::load(const rviz_common::Config & config)
{
  Display::load(config);
  marker_common_->load(config);
}

void StatusDisplay::update(MarkerUpdateArgs::WallDt wall_dt, MarkerUpdateArgs::RosDt ros_dt)
{
  marker_common_->update(wall_dt, ros_dt);
}

void StatusDisplay::reset()
{
  RTDClass::reset();
  marker_common_->clearMarkers();
}

void StatusDisplay::processMessage(crazyflie_interfaces::msg::Status::ConstSharedPtr msg)
{
  std::ostringstream text;
  text << std::fixed << std::setprecision(2) << msg->battery_voltage << "V";

  auto marker = std::make_shared<visualization_msgs::msg::Marker>();
  marker->header = msg->header;  // frame_id is the robot's frame
  marker->ns = msg->header.frame_id;
  marker->id = 0;
  marker->type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker->action = visualization_msgs::msg::Marker::ADD;
  marker->pose.position.z = 0.25;  // just above the robot
  marker->pose.orientation.w = 1.0;
  marker->scale.z = 0.08;  // text height [m]
  marker->color.r = marker->color.g = marker->color.b = marker->color.a = 1.0;
  marker->text = text.str();
  marker->frame_locked = true;  // follow the robot

  marker_common_->addMessage(marker);
}

}  // namespace crazyflie_rviz

PLUGINLIB_EXPORT_CLASS(crazyflie_rviz::StatusDisplay, rviz_common::Display)
