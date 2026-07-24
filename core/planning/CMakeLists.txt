# Global geometric planning (selectable OMPL planner: RRT* / BIT* / ABIT* / AIT* /
# EIT*) and minimum-snap trajectory generation. Third-party deps (OMPL, octomap,
# NLopt) are all ROS-free.

add_library(drone_core_planning
  src/geometric_planner.cpp
  src/min_snap_trajectory.cpp
  src/corridor.cpp
  src/corridor_trajectory.cpp)
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

# OSQP is an implementation detail of the corridor-QP solver: C API only, used
# in src/, so the include dir and library stay PRIVATE (the library is a
# concrete path — see the top-level CMakeLists — so the export carries no
# osqp target and consumers need no find_dependency).
target_include_directories(drone_core_planning PRIVATE
  ${OSQP_INCLUDE_DIR}
  ${DECOMP_UTIL_INCLUDE_DIRS})   # header-only; nothing to link
target_link_libraries(drone_core_planning PRIVATE ${OSQP_LIBRARY})

set_target_properties(drone_core_planning PROPERTIES EXPORT_NAME planning)

install(TARGETS drone_core_planning EXPORT drone_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(DRONE_CORE_BUILD_TESTS)
  add_executable(test_planner test/test_planner.cpp)
  target_link_libraries(test_planner PRIVATE drone_core_planning)
  add_test(NAME planner COMMAND test_planner)

  # Fast, map-free corridor-QP tests (analytic boxes/oracles only), kept apart
  # from the slower OMPL-driven planner test so they can run on every edit.
  add_executable(test_corridor test/test_corridor.cpp)
  target_link_libraries(test_corridor PRIVATE drone_core_planning)
  add_test(NAME corridor COMMAND test_corridor)

  # Goal projection (invalid goal -> nearest valid state). Short solve budgets
  # and analytic clearance fields, so it also runs on every planning edit.
  add_executable(test_goal_projection test/test_goal_projection.cpp)
  target_link_libraries(test_goal_projection PRIVATE drone_core_planning)
  add_test(NAME goal_projection COMMAND test_goal_projection)

  # Unknown-space surcharge (cost) + truncation's hard stop. Analytic fields and
  # predicates, so it also runs on every planning edit.
  add_executable(test_unknown_cost test/test_unknown_cost.cpp)
  target_link_libraries(test_unknown_cost PRIVATE drone_core_planning)
  add_test(NAME unknown_cost COMMAND test_unknown_cost)
endif()
