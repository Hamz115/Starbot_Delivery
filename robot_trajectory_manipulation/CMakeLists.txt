cmake_minimum_required(VERSION 3.8)
project(robot_trajectory_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)  # Added for joint states
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)


# Executables

# Planning scene
add_executable(planning_scene_node src/planning_scene.cpp)
ament_target_dependencies(planning_scene_node
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs  
  sensor_msgs          
)


add_executable(hole_placement_1
  src/hole_placement_1.cpp
)
ament_target_dependencies(hole_placement_1
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  sensor_msgs
)


add_executable(hole_placement_2
  src/hole_placement_2.cpp
)
ament_target_dependencies(hole_placement_2
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  sensor_msgs
)

add_executable(hole_placement_3
  src/hole_placement_3.cpp
)
ament_target_dependencies(hole_placement_3
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  sensor_msgs
)

add_executable(hole_placement_4
  src/hole_placement_4.cpp
)
ament_target_dependencies(hole_placement_4
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  sensor_msgs
)

add_executable(hole_placement_real_2
  src/hole_placement_real_2.cpp
)
ament_target_dependencies(hole_placement_real_2
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  sensor_msgs
)

add_executable(hole_placement_real_4
  src/hole_placement_real_4.cpp
)
ament_target_dependencies(hole_placement_real_4
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
  sensor_msgs
)


add_executable(coffee_order_handler src/coffee_order_handler.cpp)
ament_target_dependencies(coffee_order_handler
  rclcpp
  std_msgs
)

add_executable(coffee_order_handler_real src/coffee_order_handler_real.cpp)
ament_target_dependencies(coffee_order_handler_real
  rclcpp
  std_msgs
)

add_executable(planning_scene_real
  src/planning_scene_real.cpp
)
ament_target_dependencies(planning_scene_real
  rclcpp
  moveit_ros_planning_interface
  moveit_core
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  moveit_msgs
)

# Install
install(TARGETS
  planning_scene_node
  coffee_order_handler
  coffee_order_handler_real
  hole_placement_1
  hole_placement_2
  hole_placement_3
  hole_placement_4
  hole_placement_real_2
  hole_placement_real_4
  planning_scene_real
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
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
