cmake_minimum_required(VERSION 2.8.3)
project(livo_atlas)

set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Set common compile options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")

# Specific settings for Debug build
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")

# Detect CPU architecture
message(STATUS "Current CPU architecture: ${CMAKE_SYSTEM_PROCESSOR}")

# Keep the compiler's default instruction set when linking precompiled PCL and
# vikit libraries. Native ISA flags can change Eigen's allocation convention and
# make a cloud allocated inside PCL unsafe to release in this package.
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
  add_definitions(-DARM_ARCH)
else()
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funroll-loops")
  add_definitions(-DX86_ARCH)
endif()
message(STATUS "Using the compiler's default CPU instruction set")

# Define project root directory
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# Detect CPU core count for potential multithreading optimization
include(ProcessorCount)
ProcessorCount(N)
message(STATUS "Processor count: ${N}")

# Set the number of cores for multithreading
if(N GREATER 4)
  math(EXPR PROC_NUM "4")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
elseif(N GREATER 1)
  math(EXPR PROC_NUM "${N}")
  add_definitions(-DMP_EN -DMP_PROC_NUM=${PROC_NUM})
  message(STATUS "Multithreading enabled. Cores: ${PROC_NUM}")
else()
  add_definitions(-DMP_PROC_NUM=1)
  message(STATUS "Single core detected. Multithreading disabled.")
endif()

# Check for OpenMP support
find_package(OpenMP QUIET)
if(OpenMP_CXX_FOUND)
  message(STATUS "OpenMP found")
  add_compile_options(${OpenMP_CXX_FLAGS})
else()
  message(STATUS "OpenMP not found, proceeding without it")
endif()

# Check for mimalloc support
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  message(STATUS "mimalloc found")
else()
  message(STATUS "mimalloc not found, proceeding without it")
endif()

# Find catkin and required dependencies
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
  vikit_common
  vikit_ros
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

set(Sophus_LIBRARIES libSophus.so)

# Define the catkin package
catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime cv_bridge vikit_common vikit_ros image_transport
  DEPENDS EIGEN3 PCL OpenCV Sophus
)

# Include directories for dependencies
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  include
)

# Add libraries
add_library(livo_atlas_calibration STATIC
  src/sensor/extrinsics.cc src/camera/camera_config.cc)
set_target_properties(livo_atlas_calibration PROPERTIES
  POSITION_INDEPENDENT_CODE ON)
target_link_libraries(livo_atlas_calibration PUBLIC Eigen3::Eigen)
add_library(vio src/vio.cpp src/frame.cpp src/visual_point.cpp)
add_library(lio src/voxel_map.cpp)
add_library(pre src/preprocess.cpp)
add_library(imu_proc src/IMU_Processing.cpp)
add_library(livo_atlas_ros1_conversions src/ros1/measurement_conversions.cc)
target_link_libraries(livo_atlas_ros1_conversions
  ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
add_library(laser_mapping src/LIVMapper.cpp src/camera/vikit_camera_factory.cc
  src/ros1/calibration_reader.cc)
target_link_libraries(laser_mapping livo_atlas_calibration
  livo_atlas_ros1_conversions ${catkin_LIBRARIES})

# Add the main executable
add_executable(fastlivo_mapping src/main.cpp)

# Link libraries to the executable
target_link_libraries(fastlivo_mapping
  laser_mapping
  vio
  lio
  pre
  imu_proc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Link mimalloc if found
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping mimalloc)
endif()

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(measurement_conversions_test
    tests/measurement_conversions_test.cc tests/core/test_main.cc)
  if(TARGET measurement_conversions_test)
    target_link_libraries(measurement_conversions_test
      livo_atlas_ros1_conversions)
  endif()
  catkin_add_gtest(sensor_preprocess_test
    tests/sensor/preprocess_test.cc tests/core/test_main.cc)
  if(TARGET sensor_preprocess_test)
    target_link_libraries(sensor_preprocess_test pre ${OpenMP_CXX_LIBRARIES}
      ${catkin_LIBRARIES} ${PCL_LIBRARIES})
  endif()
  catkin_add_gtest(pcl_memory_compatibility_test tests/pcl_memory_compatibility_test.cc)
  if(TARGET pcl_memory_compatibility_test)
    target_link_libraries(pcl_memory_compatibility_test ${PCL_LIBRARIES})
  endif()
  catkin_add_gtest(core_math_state_test
    tests/core/math_test.cc tests/core/state_test.cc tests/core/test_main.cc)
  catkin_add_gtest(calibration_test
    tests/core/calibration_test.cc tests/core/test_main.cc)
  if(TARGET calibration_test)
    target_link_libraries(calibration_test livo_atlas_calibration)
  endif()
  catkin_add_gtest(legacy_core_compatibility_test
    tests/legacy_core_compatibility_test.cc tests/core/test_main.cc)
  catkin_add_gtest(vio_reference_patch_test
    tests/vio_reference_patch_test.cc tests/core/test_main.cc)
  catkin_add_gtest(vio_sampling_test
    tests/vio_sampling_test.cc tests/core/test_main.cc)
  catkin_add_gtest(camera_config_test
    tests/camera_config_test.cc tests/core/test_main.cc)
  if(TARGET camera_config_test)
    target_link_libraries(camera_config_test laser_mapping vio lio pre imu_proc
      ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES}
      ${Sophus_LIBRARIES} ${OpenMP_CXX_LIBRARIES})
  endif()
  foreach(VIO_TEST vio_reference_patch_test vio_sampling_test)
    if(TARGET ${VIO_TEST})
      target_link_libraries(${VIO_TEST} vio lio ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES} ${Sophus_LIBRARIES} ${OpenMP_CXX_LIBRARIES})
    endif()
  endforeach()
endif()
