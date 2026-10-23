cmake_minimum_required(VERSION 3.10)
project(livo_atlas_frontend_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(GTest REQUIRED)
enable_testing()

get_filename_component(PACKAGE_INCLUDE_DIR "../../include" ABSOLUTE)
add_library(synchronization STATIC ../../src/sensor/synchronizer.cc)
target_include_directories(synchronization PUBLIC
  "${PACKAGE_INCLUDE_DIR}" ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(synchronization PUBLIC Eigen3::Eigen
  ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(synchronizer_test synchronizer_test.cc ../core/test_main.cc)
target_link_libraries(synchronizer_test PRIVATE synchronization GTest::GTest)
add_test(NAME synchronizer_test COMMAND synchronizer_test)

set(HEADER_CHECK_SOURCES)
foreach(HEADER frontend/synchronized_measurements sensor/synchronizer)
  string(REPLACE "/" "_" CHECK_NAME "${HEADER}")
  set(CHECK_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/${CHECK_NAME}_header_check.cc")
  file(WRITE "${CHECK_SOURCE}" "#include \"livo_atlas/${HEADER}.h\"\n")
  list(APPEND HEADER_CHECK_SOURCES "${CHECK_SOURCE}")
endforeach()
add_library(frontend_header_checks OBJECT ${HEADER_CHECK_SOURCES})
target_include_directories(frontend_header_checks PRIVATE
  "${PACKAGE_INCLUDE_DIR}" ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
