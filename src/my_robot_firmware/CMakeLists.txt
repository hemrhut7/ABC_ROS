cmake_minimum_required(VERSION 3.8)
project(my_robot_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(esp32_serial_node src/esp32_serial_node.cpp)
target_include_directories(esp32_serial_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(esp32_serial_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

add_executable(data_logger_node src/data_logger_node.cpp)
target_include_directories(data_logger_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(data_logger_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
)

install(TARGETS
  esp32_serial_node
  data_logger_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
