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
find_package(visualization_msgs REQUIRED)
find_package(DecompUtil REQUIRED)

# Add the new optimizer library alongside the existing planner
add_library(ompl_planner_lib src/ompl_rrt_core.cpp)
add_library(sfc_optimizer_lib src/sfc_optimizer.cpp)
add_executable(global_planner_node src/planner_node.cpp)

# Include directories for the planner library
target_include_directories(ompl_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OMPL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Include directories for the optimizer library
target_include_directories(sfc_optimizer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${DecompUtil_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Include directories for the main executable node
target_include_directories(global_planner_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

# Link external libraries to the planner
target_link_libraries(ompl_planner_lib 
  ${OMPL_LIBRARIES} 
  ${OCTOMAP_LIBRARIES}
)

# Link external libraries to the optimizer
target_link_libraries(sfc_optimizer_lib 
  ${DecompUtil_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)

# Link internal libraries to the main executable node
target_link_libraries(global_planner_node 
  ompl_planner_lib
  sfc_optimizer_lib
)

# Link ROS dependencies to the main executable node
ament_target_dependencies(global_planner_node
  rclcpp
  octomap_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
)

# Install rules for all targets
install(TARGETS
  ompl_planner_lib
  sfc_optimizer_lib
  global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()