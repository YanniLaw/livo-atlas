cmake_minimum_required(VERSION 3.10)
project(livo_atlas_io_tests LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs)
find_package(GTest REQUIRED)
enable_testing()

get_filename_component(PACKAGE_INCLUDE_DIR "../../include" ABSOLUTE)
add_library(livo_exporter STATIC ../../src/io/livo_exporter.cc
  ../../src/camera/camera.cc ../../src/camera/camera_config.cc
  ../../src/camera/camera_factory.cc ../../src/camera/pinhole_camera.cc
  ../../src/camera/equidistant_camera.cc ../../src/image/image_ops.cc)
target_include_directories(livo_exporter PUBLIC
  "${PACKAGE_INCLUDE_DIR}" ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(livo_exporter PUBLIC Eigen3::Eigen
  ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})
add_executable(livo_exporter_test livo_exporter_test.cc ../test_main.cc)
target_link_libraries(livo_exporter_test PRIVATE livo_exporter GTest::GTest)
add_test(NAME livo_exporter_test COMMAND livo_exporter_test)

foreach(HEADER frontend/diagnostics frontend/frontend_result
    mapping/cloud_colorizer io/livo_exporter)
  string(REPLACE "/" "_" CHECK_NAME "${HEADER}")
  set(CHECK_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/${CHECK_NAME}_header_check.cc")
  file(WRITE "${CHECK_SOURCE}" "#include \"livo_atlas/${HEADER}.h\"\n")
  add_library(${CHECK_NAME}_header_check OBJECT "${CHECK_SOURCE}")
  target_include_directories(${CHECK_NAME}_header_check PRIVATE
    "${PACKAGE_INCLUDE_DIR}" ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
endforeach()
