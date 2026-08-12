cmake_minimum_required(VERSION 3.16)
project(spar_perception C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  DEPENDENCIES std_msgs geometry_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(anomaly_detector src/anomaly_detector.cpp)
target_link_libraries(anomaly_detector PRIVATE
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  opencv_core
  opencv_imgproc
  ${sensor_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  "${cpp_typesupport_target}")

install(TARGETS anomaly_detector DESTINATION lib/${PROJECT_NAME})

ament_package()
