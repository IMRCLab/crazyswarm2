cmake_minimum_required(VERSION 3.10)
project(crazyflie_server_cpp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_capture_tracking_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ros_environment REQUIRED)

if (DEFINED ENV{ROS_DISTRO})
  set(ROS_DISTRO_VALUE "$ENV{ROS_DISTRO}")
else()
  message(WARNING "ROS_DISTRO environment variable not set.")
  set(ROS_DISTRO_VALUE "unknown_distro")
endif()

add_subdirectory(deps/crazyflie_tools)

include_directories(
  deps/crazyflie_tools/crazyflie_cpp/include
  deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/include
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(crazyflie_server
  src/crazyflie_server.cpp
)

if("${ROS_DISTRO_VALUE}" STREQUAL "humble")
  target_compile_definitions(crazyflie_server PRIVATE ROS_DISTRO_HUMBLE)
endif()

target_link_libraries(crazyflie_server
  crazyflie_cpp
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${crazyflie_interfaces_TARGETS}
  ${motion_capture_tracking_interfaces_TARGETS}
)

install(TARGETS
  comCheck
  scan
  listParams
  listLogVariables
  listMemories
  reboot
  battery
  version
  console
  log
  setParam
  downloadUSDLogfile
  #
  crazyflie_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
