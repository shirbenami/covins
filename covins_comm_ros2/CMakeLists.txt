cmake_minimum_required(VERSION 3.5)
project(covins_comm_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(covins_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

set(ROS2_SOURCE_FILES
  src/ros_conversion.cpp
  src/comm_ros2_bridge.cpp
  # add other ROS2-specific sources here
)

add_library(${PROJECT_NAME} SHARED
  ${ROS2_SOURCE_FILES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  covins_msgs
  Eigen3
  OpenCV
  geometry_msgs
  std_msgs
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp covins_msgs Eigen3 OpenCV geometry_msgs std_msgs)
ament_package()