cmake_minimum_required(VERSION 3.8)
project(fire_drone_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
add_executable(fire_nav2_planner
  src/fire_nav2_planner.cpp
)

ament_target_dependencies(fire_nav2_planner
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
)

add_executable(fire_frontier_planner
  src/fire_frontier_planner.cpp
)
add_executable(height_filtered_map_node
  src/height_filtered_map_node.cpp
)

ament_target_dependencies(height_filtered_map_node
  rclcpp
  nav_msgs
  sensor_msgs
)
ament_target_dependencies(fire_frontier_planner
  rclcpp
  geometry_msgs
  nav_msgs
    tf2
  tf2_ros
)

install(TARGETS
  fire_nav2_planner
  fire_frontier_planner
   height_filtered_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
