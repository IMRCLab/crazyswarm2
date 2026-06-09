cmake_minimum_required(VERSION 3.8)
project(crazyflie)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclpy REQUIRED)
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
  message(WARNING "ROS_DISTRO environment variable not set. C++ macro will be empty or default.")
  set(ROS_DISTRO_VALUE "unknown_distro") # Or an empty string, or "NO_ROS_DISTRO_SET"
endif()

# add dependencies
add_subdirectory(deps/crazyflie_tools)
# add_subdirectory(externalDependencies/libmotioncapture)

include_directories(
  deps/crazyflie_tools/crazyflie_cpp/include
  deps/crazyflie_tools/crazyflie_cpp/crazyflie-link-cpp/include
#  externalDependencies/libobjecttracker/include
#  externalDependencies/libmotioncapture/include
  ${EIGEN3_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
)

add_executable(teleop 
  src/teleop.cpp
)
target_link_libraries(teleop 
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${crazyflie_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
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

# # scan
# add_executable(scan
#   src/scan.cpp
# )
# target_link_libraries(scan
#   crazyflie_cpp
#   ${Boost_LIBRARIES}
# )
# target_compile_features(scan PUBLIC c_std_99 cxx_std_17)

# Install C++ executables
install(TARGETS
  # crazyflie_tools
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
  teleop
  crazyflie_server
  DESTINATION lib/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/chooser.py
  scripts/cfmult.py
  scripts/aideck_streamer.py
  scripts/flash.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files.
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
