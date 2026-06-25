cmake_minimum_required(VERSION 3.16)
project(drone_core VERSION 0.1.0 LANGUAGES CXX)

# The autonomy core is a plain CMake project: it builds and unit-tests with no
# ROS in the environment. The ament wrappers consume it via find_package after
# it is installed to a prefix.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(DRONE_CORE_BUILD_TESTS "Build the drone_core unit tests" ON)

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(octomap REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)

if(DRONE_CORE_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(common)
add_subdirectory(control)
add_subdirectory(planning)
add_subdirectory(autonomy)

# Export package so ament wrappers can find_package(drone_core CONFIG).
include(CMakePackageConfigHelpers)

install(EXPORT drone_coreTargets
  FILE drone_coreTargets.cmake
  NAMESPACE drone_core::
  DESTINATION lib/cmake/drone_core)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/drone_coreConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/drone_coreConfig.cmake
  INSTALL_DESTINATION lib/cmake/drone_core)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/drone_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/drone_coreConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/drone_coreConfigVersion.cmake
  DESTINATION lib/cmake/drone_core)
