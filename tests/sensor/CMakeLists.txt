cmake_minimum_required(VERSION 3.10)
project(livo_atlas_sensor_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED COMPONENTS core)
find_package(OpenMP REQUIRED)
find_package(GTest REQUIRED)
enable_testing()

get_filename_component(PACKAGE_INCLUDE_DIR "../../include" ABSOLUTE)
add_library(sensor_preprocess STATIC ../../src/sensor/lidar_preprocessor.cc)
target_include_directories(sensor_preprocess PUBLIC
  "${PACKAGE_INCLUDE_DIR}" ${PCL_INCLUDE_DIRS})
target_link_libraries(sensor_preprocess PUBLIC Eigen3::Eigen
  ${PCL_COMMON_LIBRARIES} OpenMP::OpenMP_CXX)
add_executable(sensor_preprocess_test preprocess_test.cc ../test_main.cc)
target_link_libraries(sensor_preprocess_test PRIVATE
  sensor_preprocess GTest::GTest)
add_test(NAME sensor_preprocess_test COMMAND sensor_preprocess_test)

set(HEADER_CHECK_SOURCES)
foreach(HEADER point_types measurements lidar_scan)
  set(CHECK_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/${HEADER}_header_check.cc")
  file(WRITE "${CHECK_SOURCE}" "#include \"livo_atlas/sensor/${HEADER}.h\"\n")
  list(APPEND HEADER_CHECK_SOURCES "${CHECK_SOURCE}")
endforeach()
add_library(sensor_header_checks OBJECT ${HEADER_CHECK_SOURCES})
target_include_directories(sensor_header_checks PRIVATE
  "${PACKAGE_INCLUDE_DIR}" ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
