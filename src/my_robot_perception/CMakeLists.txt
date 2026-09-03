cmake_minimum_required(VERSION 3.8)
project(my_robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(include)

# Component Library: image_splitter_component
add_library(image_splitter_component SHARED
  src/image_splitter_node.cpp
)
target_include_directories(image_splitter_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(image_splitter_component
  ${OpenCV_LIBS}
)
ament_target_dependencies(image_splitter_component
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
)
rclcpp_components_register_nodes(image_splitter_component "my_robot_perception::ImageSplitterNode")

# Standalone Executable 1: image_splitter_node
add_executable(image_splitter_node src/image_splitter_node_main.cpp)
target_link_libraries(image_splitter_node
  image_splitter_component
)
ament_target_dependencies(image_splitter_node
  rclcpp
)

# Standalone Executable 2: n10_lidar_node
add_executable(n10_lidar_node src/n10_lidar_node.cpp)
target_include_directories(n10_lidar_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(n10_lidar_node
  rclcpp
  sensor_msgs
  std_msgs
)

# Install
install(TARGETS
  image_splitter_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  image_splitter_node
  n10_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_export_include_directories(include)
ament_export_libraries(image_splitter_component)
ament_export_dependencies(
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
  cv_bridge
  OpenCV
)

ament_package()
