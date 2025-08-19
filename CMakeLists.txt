cmake_minimum_required(VERSION 3.8)
project(lattice_planner_pkg)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(planning_custom_msgs REQUIRED)
find_package(obstacle_detection_pkg REQUIRED)

# Note: Messages moved to planning_custom_msgs package

# Lattice Planner Node (reorganized structure)
add_executable(lattice_planner_node 
  src/planner/lattice_planner_node.cpp
  src/planner/lattice_planner.cpp
  src/core/frenet_coordinate.cpp
  src/core/path_generator.cpp
  src/core/obstacle_detector.cpp
  src/core/spline_utils.cpp
  src/obstacle_detector.cpp
  src/obstacle_subscriber.cpp
  src/path_selector.cpp
)

ament_target_dependencies(lattice_planner_node
  rclcpp 
  nav_msgs 
  geometry_msgs 
  std_msgs
  visualization_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  planning_custom_msgs
  obstacle_detection_pkg
)

# Note: Using planning_custom_msgs for message types

# Messages will be automatically linked through ament_target_dependencies

# Include directories for organized structure
target_include_directories(lattice_planner_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Obstacle Subscriber Node (standalone)
add_executable(obstacle_subscriber_node 
  src/obstacle_subscriber_node.cpp
  src/obstacle_subscriber.cpp
)

ament_target_dependencies(obstacle_subscriber_node
  rclcpp 
  nav_msgs 
  geometry_msgs 
  std_msgs
  visualization_msgs
  obstacle_detection_pkg
)

target_include_directories(obstacle_subscriber_node
  PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Install targets
install(TARGETS 
        lattice_planner_node
        obstacle_subscriber_node
        DESTINATION lib/${PROJECT_NAME})

# Install header files
install(DIRECTORY include/
        DESTINATION include/)

# Install config files
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config)

# Install launch files
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)

# Export runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

ament_package()