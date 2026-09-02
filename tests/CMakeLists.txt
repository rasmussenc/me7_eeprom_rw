# Unit tests. A tiny dependency-free assert-based harness driven by CTest.
# Can be swapped for Catch2/GoogleTest later without touching tests.

add_executable(test_options unit/test_options.cpp)
target_link_libraries(test_options PRIVATE me7_core)
add_test(NAME test_options COMMAND test_options)

add_executable(test_core_logic
  unit/test_core_logic.cpp
  ${CMAKE_SOURCE_DIR}/src/firmware/embedded.cpp
)
target_include_directories(test_core_logic PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(test_core_logic PRIVATE me7_core)
add_test(NAME test_core_logic COMMAND test_core_logic)
