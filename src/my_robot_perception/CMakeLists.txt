cmake_minimum_required(VERSION 3.8)
project(my_robot_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Executable 1: image_splitter_node
add_executable(image_splitter_node src/image_splitter_node.cpp)
target_include_directories(image_splitter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(image_splitter_node
  ${OpenCV_LIBS}
)
ament_target_dependencies(image_splitter_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# Executable 2: n10_lidar_node
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

install(TARGETS
  image_splitter_node
  n10_lidar_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
