cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ompl REQUIRED)
find_package(octomap REQUIRED)

# Define the targets
add_library(ompl_planner_lib src/ompl_rrt_core.cpp)
add_executable(global_planner_node src/planner_node.cpp)

# Point to the include root (not the subfolder)
target_include_directories(ompl_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OMPL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_include_directories(global_planner_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link math and system libraries
target_link_libraries(ompl_planner_lib 
  ${OMPL_LIBRARIES} 
  ${OCTOMAP_LIBRARIES}
)

target_link_libraries(global_planner_node ompl_planner_lib)

# Link ROS dependencies
ament_target_dependencies(global_planner_node
  rclcpp
  octomap_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS
  ompl_planner_lib
  global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()