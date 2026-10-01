cmake_minimum_required(VERSION 3.8)
project(hcr3a_pick_place)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(pick_place_node src/pick_place_node.cpp)
target_compile_features(pick_place_node PUBLIC cxx_std_17)
ament_target_dependencies(pick_place_node
  rclcpp geometry_msgs moveit_msgs moveit_core moveit_ros_planning_interface shape_msgs)

install(TARGETS pick_place_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
