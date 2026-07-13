# Top-level orchestrator: composes planning and control, runs the planner on a
# worker thread, and exposes the fast control step.

find_package(Threads REQUIRED)

add_library(drone_core_autonomy src/autonomy_core.cpp)
add_library(drone_core::autonomy ALIAS drone_core_autonomy)

target_include_directories(drone_core_autonomy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(drone_core_autonomy PUBLIC
  drone_core_control
  drone_core_planning
  drone_core_common
  Threads::Threads
  ${DYNAMICEDT3D_LIBRARY})

set_target_properties(drone_core_autonomy PROPERTIES EXPORT_NAME autonomy)

install(TARGETS drone_core_autonomy EXPORT drone_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(DRONE_CORE_BUILD_TESTS)
  add_executable(test_autonomy_core test/test_autonomy_core.cpp)
  target_link_libraries(test_autonomy_core PRIVATE drone_core_autonomy)
  add_test(NAME autonomy_core COMMAND test_autonomy_core)
endif()
