cmake_minimum_required(VERSION 3.16)
project(spar_air CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(spar_perception REQUIRED)

add_executable(bt_executive src/bt_executive.cpp)
target_include_directories(bt_executive PRIVATE src)
target_link_libraries(bt_executive PRIVATE
  behaviortree_cpp::behaviortree_cpp
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${px4_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${std_msgs_TARGETS}
  ${spar_perception_TARGETS})

# PX4 estimate -> TF, so the detector projects pixels through the estimated
# pose, same honesty as the ground demo (no ground-truth TF from the sim).
add_executable(tf_from_px4 src/tf_from_px4.cpp)
target_include_directories(tf_from_px4 PRIVATE src)
target_link_libraries(tf_from_px4 PRIVATE
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  ${px4_msgs_TARGETS}
  ${geometry_msgs_TARGETS})

install(TARGETS bt_executive tf_from_px4
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees/ DESTINATION share/${PROJECT_NAME}/behavior_trees)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
