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
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)

# define targets
add_library(ompl_planner_lib src/ompl_rrt_core.cpp)
add_library(trajectory_optimizer_lib src/trajectory_optimizer.cpp)
add_library(osqp_trajectory_optimizer_lib src/osqp_trajectory_optimizer.cpp)
add_library(nlopt_trajectory_optimizer_lib src/nlopt_trajectory_optimizer.cpp)
add_executable(global_planner_node src/planner_node.cpp)

# configure include directories
target_include_directories(ompl_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${OMPL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_include_directories(trajectory_optimizer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(global_planner_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_include_directories(osqp_trajectory_optimizer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIR}
)

target_include_directories(nlopt_trajectory_optimizer_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${NLOPT_INCLUDE_DIRS}
)

# link math and external libraries
target_link_libraries(ompl_planner_lib 
  ${OMPL_LIBRARIES} 
  ${OCTOMAP_LIBRARIES}
)

target_link_libraries(trajectory_optimizer_lib
  Eigen3::Eigen
)

target_link_libraries(osqp_trajectory_optimizer_lib
  Eigen3::Eigen
)

target_link_libraries(nlopt_trajectory_optimizer_lib
  ${NLOPT_LIBRARIES}
  trajectory_optimizer_lib
)

target_link_libraries(global_planner_node 
  ompl_planner_lib
  trajectory_optimizer_lib
  osqp_trajectory_optimizer_lib
  nlopt_trajectory_optimizer_lib
)

# link ros dependencies
ament_target_dependencies(osqp_trajectory_optimizer_lib
  osqp_vendor
)

ament_target_dependencies(global_planner_node
  rclcpp
  octomap_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  osqp_vendor
)

ament_target_dependencies(nlopt_trajectory_optimizer_lib
  rclcpp
)

# set install destinations
install(TARGETS
  ompl_planner_lib
  trajectory_optimizer_lib
  osqp_trajectory_optimizer_lib
  nlopt_trajectory_optimizer_lib
  global_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()