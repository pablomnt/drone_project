# Shared types, frame conversions and logging. Header-heavy and dependency-light:
# everything here speaks Eigen and nothing else.

add_library(drone_core_common src/frames.cpp)
add_library(drone_core::common ALIAS drone_core_common)

target_include_directories(drone_core_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(drone_core_common PUBLIC Eigen3::Eigen)

set_target_properties(drone_core_common PROPERTIES EXPORT_NAME common)

install(TARGETS drone_core_common EXPORT drone_coreTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)

if(DRONE_CORE_BUILD_TESTS)
  add_executable(test_frames test/test_frames.cpp)
  target_link_libraries(test_frames PRIVATE drone_core_common)
  add_test(NAME frames COMMAND test_frames)
endif()
