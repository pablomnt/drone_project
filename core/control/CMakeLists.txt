# The position controller. Pure Eigen maths; knows nothing about ROS or PX4.

add_library(drone_core_control
  src/position_control.cpp
  src/flatness_mapper.cpp
  src/trajectory_tracker.cpp)
add_library(drone_core::control ALIAS drone_core_control)

target_include_directories(drone_core_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(drone_core_control PUBLIC drone_core_common Eigen3::Eigen)

set_target_properties(drone_core_control PROPERTIES EXPORT_NAME control)

install(TARGETS drone_core_control EXPORT drone_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(DRONE_CORE_BUILD_TESTS)
  add_executable(test_position_control test/test_position_control.cpp)
  target_link_libraries(test_position_control PRIVATE drone_core_control)
  add_test(NAME position_control COMMAND test_position_control)

  add_executable(test_feedforward test/test_feedforward.cpp)
  target_link_libraries(test_feedforward PRIVATE drone_core_control)
  add_test(NAME feedforward COMMAND test_feedforward)

  add_executable(test_flatness_mapper test/test_flatness_mapper.cpp)
  target_link_libraries(test_flatness_mapper PRIVATE drone_core_control)
  add_test(NAME flatness_mapper COMMAND test_flatness_mapper)
endif()
