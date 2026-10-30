cmake_minimum_required(VERSION 3.10)
project(crazyflie)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(crazyflie_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(teleop 
  src/teleop.cpp
)
target_link_libraries(teleop 
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${crazyflie_interfaces_TARGETS}
  ${geometry_msgs_TARGETS}
  Eigen3::Eigen
)

# Install C++ executables
install(TARGETS
  teleop
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
