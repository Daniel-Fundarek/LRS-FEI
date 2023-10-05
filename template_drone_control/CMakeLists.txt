cmake_minimum_required(VERSION 3.5)
project(template_drone_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

include_directories(
  include
  /opt/ros/foxy/include
)


# Declare a C++ executable
add_executable(drone_control_node src/drone_control_node.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(drone_control_node rclcpp geometry_msgs mavros_msgs)

# Install the executable
install(TARGETS drone_control_node DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()
