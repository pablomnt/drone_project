cmake_minimum_required(VERSION 3.8)
project(pc_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find the PCL library installed on your system
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})

# Tell CMake to build your node
add_executable(dataset_pc_node src/dataset_pc_node.cpp)

# Link the PCL libraries to your node
target_link_libraries(dataset_pc_node ${PCL_LIBRARIES})
ament_target_dependencies(dataset_pc_node rclcpp sensor_msgs pcl_conversions)

# Install the executable so ROS 2 can find it
install(TARGETS
  dataset_pc_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
