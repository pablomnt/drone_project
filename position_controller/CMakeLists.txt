cmake_minimum_required(VERSION 3.8)
project(position_controller)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)        # <--- This is what you actually need
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)

# 2. Include Directories
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
)

# 3. Create the Math Library
add_library(position_control_lib src/PositionControl.cpp)
target_link_libraries(position_control_lib Eigen3::Eigen)

# 4. Create the ROS 2 Node
add_executable(position_controller_node src/position_controller_node.cpp)
target_link_libraries(position_controller_node 
  position_control_lib
  ${Eigen3_LIBRARIES}
)
ament_target_dependencies(position_controller_node
  rclcpp
  px4_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# 5. Create the Test Executable (Optional, but good to keep)
add_executable(test_library src/test_library.cpp)
target_link_libraries(test_library position_control_lib)

# 6. Install Rules
install(TARGETS 
  position_control_lib
  position_controller_node
  test_library
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()