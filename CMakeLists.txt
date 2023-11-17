#######################################################################################
cmake_minimum_required(VERSION 3.8)
#######################################################################################
project(t07_robot)
set(T07_ROBOT_TARGET ${PROJECT_NAME}_node)
#######################################################################################
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()
#######################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
##########################################################################
add_subdirectory(external/107-Arduino-Cyphal)
add_subdirectory(external/libsocketcan)
add_subdirectory(external/mp-units/src)
#######################################################################################
include_directories(
  include
)
#######################################################################################
add_executable(${T07_ROBOT_TARGET}
  src/CanManager.cpp
  src/Node.cpp
  src/main.cpp
)
#######################################################################################
target_link_libraries(${T07_ROBOT_TARGET}
  cyphal++ socketcan mp-units
)
#######################################################################################
target_compile_features(${T07_ROBOT_TARGET} PRIVATE cxx_std_20)
ament_target_dependencies(${T07_ROBOT_TARGET} rclcpp std_msgs)
#######################################################################################
install(TARGETS
        ${T07_ROBOT_TARGET}
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
        launch
        DESTINATION share/${PROJECT_NAME}
)
#######################################################################################
ament_package()
#######################################################################################
