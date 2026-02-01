cmake_minimum_required(VERSION 3.8)
project(pruebas_pablo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Incluir directorios
include_directories(
  include
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


# Añadir ejecutable
add_executable(prueba2
  src/prueba2.cpp
)
add_executable(prueba2_waypoints_node
  src/prueba2_waypoints.cpp
)
add_executable(prueba3_BC
  src/prueba3_BC.cpp
)

# Instalar
install(TARGETS
  prueba2
  prueba2_waypoints_node
  prueba3_BC
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_target_dependencies(prueba2
  rclcpp
  px4_msgs
  nav_msgs
)
ament_target_dependencies(prueba2_waypoints_node
  rclcpp
  px4_msgs
)
ament_target_dependencies(prueba3_BC
  rclcpp
  px4_msgs
  nav_msgs
)

ament_python_install_package(${PROJECT_NAME})
ament_package()
