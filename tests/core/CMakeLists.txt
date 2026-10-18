cmake_minimum_required(VERSION 3.10)
project(livo_atlas_core_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(GTest REQUIRED)
enable_testing()

get_filename_component(PACKAGE_INCLUDE_DIR "../../include" ABSOLUTE)
add_executable(core_math_state_test math_test.cc state_test.cc test_main.cc)
target_include_directories(core_math_state_test PRIVATE "${PACKAGE_INCLUDE_DIR}")
target_link_libraries(core_math_state_test PRIVATE Eigen3::Eigen GTest::GTest)
add_test(NAME core_math_state_test COMMAND core_math_state_test)

set(HEADER_CHECK_SOURCES)
foreach(HEADER math_types so3_math state)
  set(CHECK_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/${HEADER}_header_check.cc")
  file(WRITE "${CHECK_SOURCE}" "#include \"livo_atlas/core/${HEADER}.h\"\n")
  list(APPEND HEADER_CHECK_SOURCES "${CHECK_SOURCE}")
endforeach()
add_library(core_header_checks OBJECT ${HEADER_CHECK_SOURCES})
target_include_directories(core_header_checks PRIVATE "${PACKAGE_INCLUDE_DIR}")
# Object libraries cannot use target_link_libraries before CMake 3.12.
get_target_property(CORE_EIGEN_INCLUDE_DIRS Eigen3::Eigen
  INTERFACE_INCLUDE_DIRECTORIES)
target_include_directories(core_header_checks SYSTEM PRIVATE
  ${CORE_EIGEN_INCLUDE_DIRS})
