# Global geometric planning (selectable OMPL planner: RRT* / BIT* / ABIT* / AIT* /
# EIT*) and minimum-snap trajectory generation. Third-party deps (OMPL, octomap,
# NLopt) are all ROS-free.

add_library(drone_core_planning
  src/geometric_planner.cpp
  src/min_snap_trajectory.cpp)
add_library(drone_core::planning ALIAS drone_core_planning)

target_include_directories(drone_core_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${NLOPT_INCLUDE_DIRS})

target_link_libraries(drone_core_planning PUBLIC
  drone_core_common
  Eigen3::Eigen
  ${OMPL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${NLOPT_LIBRARIES})

set_target_properties(drone_core_planning PROPERTIES EXPORT_NAME planning)

install(TARGETS drone_core_planning EXPORT drone_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(DRONE_CORE_BUILD_TESTS)
  add_executable(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner PRIVATE drone_core_planning)
  add_test(NAME planner COMMAND test_planner)
endif()
