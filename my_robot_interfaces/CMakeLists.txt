cmake_minimum_required(VERSION 3.8)
project(my_robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces (empty list for now, user can add .msg/.srv files later)
rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/CustomTelemetry.msg"
  # "srv/CustomCommand.srv"
)

ament_package()
