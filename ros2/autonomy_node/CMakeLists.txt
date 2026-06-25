cmake_minimum_required(VERSION 3.8)
project(autonomy_node)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(drone_interfaces REQUIRED)

# The ROS-free autonomy core, installed to a prefix on CMAKE_PREFIX_PATH.
find_package(drone_core REQUIRED CONFIG)

add_executable(autonomy_node src/autonomy_node.cpp)

target_include_directories(autonomy_node PRIVATE ${OCTOMAP_INCLUDE_DIRS})

target_link_libraries(autonomy_node
  drone_core::autonomy
  drone_core::common
  ${OCTOMAP_LIBRARIES})

ament_target_dependencies(autonomy_node
  rclcpp
  px4_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  octomap_msgs
  drone_interfaces)

install(TARGETS autonomy_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
