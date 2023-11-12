#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(t07_ros)
set(T07_ROS_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
#######################################################################################
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
##########################################################################
add_subdirectory(external/107-Arduino-Cyphal)
add_subdirectory(external/libsocketcan)
#######################################################################################
include_directories(
  include
)
#######################################################################################
add_executable(${T07_ROS_TARGET}
  src/CanManager.cpp
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_link_libraries(${T07_ROS_TARGET}
  cyphal++ socketcan
)
#######################################################################################
target_compile_features(${T07_ROS_TARGET} PRIVATE cxx_std_17)
ament_target_dependencies(${T07_ROS_TARGET} rclcpp)
#######################################################################################
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
#######################################################################################
ament_package()
#######################################################################################