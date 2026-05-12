cmake_minimum_required(VERSION 3.8)
project(fire_drone_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(fire_drone_msgs REQUIRED)
add_executable(offboard_controller src/offboard_controller.cpp)
add_executable(fire_rtabmap_planner src/fire_rtabmap_planner.cpp)
add_executable(nav2_path_follower
  src/nav2_path_follower.cpp
)

ament_target_dependencies(nav2_path_follower
  rclcpp
  nav_msgs
  px4_msgs
)
ament_target_dependencies(fire_rtabmap_planner
  rclcpp
  geometry_msgs
  nav_msgs
)
ament_target_dependencies(offboard_controller
  rclcpp
  px4_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  fire_drone_msgs
)

install(TARGETS
  offboard_controller
  fire_rtabmap_planner
  nav2_path_follower
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
