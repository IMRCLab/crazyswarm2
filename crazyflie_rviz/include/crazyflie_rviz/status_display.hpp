#ifndef CRAZYFLIE_RVIZ__STATUS_DISPLAY_HPP_
#define CRAZYFLIE_RVIZ__STATUS_DISPLAY_HPP_

#include <memory>

#include "crazyflie_interfaces/msg/status.hpp"
#include "rviz_common/ros_topic_display.hpp"
#include "rviz_default_plugins/displays/marker/marker_common.hpp"

namespace crazyflie_rviz
{

/// Shows the status of a Crazyflie (currently the battery voltage) on top of the robot.
class StatusDisplay : public rviz_common::RosTopicDisplay<crazyflie_interfaces::msg::Status>
{
  Q_OBJECT

public:
  StatusDisplay();

  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void update(std::chrono::nanoseconds wall_dt, std::chrono::nanoseconds ros_dt) override;
  void reset() override;

protected:
  void processMessage(crazyflie_interfaces::msg::Status::ConstSharedPtr msg) override;

private:
  std::unique_ptr<rviz_default_plugins::displays::MarkerCommon> marker_common_;
};

}  // namespace crazyflie_rviz

#endif  // CRAZYFLIE_RVIZ__STATUS_DISPLAY_HPP_
