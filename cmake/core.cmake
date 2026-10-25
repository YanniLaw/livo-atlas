find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d imgcodecs)
find_package(Sophus REQUIRED)
find_package(Threads REQUIRED)
include(${CMAKE_CURRENT_LIST_DIR}/sophus_target.cmake)
if(LIVO_ATLAS_USE_OPENMP)
  find_package(OpenMP QUIET)
endif()

add_library(livo_atlas_build_options INTERFACE)
target_compile_features(livo_atlas_build_options INTERFACE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(livo_atlas_build_options INTERFACE -fexceptions)
  if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64|ARM|AARCH64)")
    target_compile_options(livo_atlas_build_options INTERFACE
      $<$<CONFIG:Release>:-funroll-loops>)
  endif()
endif()
target_link_libraries(livo_atlas_build_options INTERFACE Threads::Threads)

function(livo_atlas_library name)
  add_library(livo_atlas_${name} ${ARGN})
  set_target_properties(livo_atlas_${name} PROPERTIES
    EXPORT_NAME ${name} INSTALL_RPATH_USE_LINK_PATH TRUE)
  if(UNIX AND NOT APPLE)
    set_target_properties(livo_atlas_${name} PROPERTIES INSTALL_RPATH "$ORIGIN")
  elseif(APPLE)
    set_target_properties(livo_atlas_${name} PROPERTIES INSTALL_RPATH "@loader_path")
  endif()
  add_library(livo_atlas::${name} ALIAS livo_atlas_${name})
  target_include_directories(livo_atlas_${name} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(livo_atlas_${name} PUBLIC livo_atlas_build_options)
endfunction()

livo_atlas_library(calibration src/sensor/extrinsics.cc src/camera/camera_config.cc)
target_link_libraries(livo_atlas_calibration PUBLIC Eigen3::Eigen)
livo_atlas_library(keyframes src/mapping/keyframe.cc src/mapping/keyframe_selector.cc)
target_link_libraries(livo_atlas_keyframes PUBLIC livo_atlas_calibration pcl_common opencv_core)
livo_atlas_library(camera src/camera/camera.cc src/camera/camera_factory.cc
  src/camera/pinhole_camera.cc src/camera/equidistant_camera.cc src/image/image_ops.cc)
target_link_libraries(livo_atlas_camera PUBLIC livo_atlas_calibration
  opencv_core opencv_imgproc opencv_calib3d)
livo_atlas_library(synchronization src/sensor/synchronizer.cc)
target_link_libraries(livo_atlas_synchronization PUBLIC Eigen3::Eigen pcl_common opencv_core)
livo_atlas_library(preprocess src/sensor/lidar_preprocessor.cc)
target_link_libraries(livo_atlas_preprocess PUBLIC Eigen3::Eigen pcl_common)
livo_atlas_library(imu src/frontend/imu/imu_processor.cc)
target_link_libraries(livo_atlas_imu PUBLIC Eigen3::Eigen pcl_common opencv_core Sophus::Sophus)
livo_atlas_library(lio src/mapping/voxel_map.cc)
target_link_libraries(livo_atlas_lio PUBLIC Eigen3::Eigen pcl_filters opencv_core Sophus::Sophus)
livo_atlas_library(vio src/frontend/vision/vio.cc src/frontend/vision/frame.cc
  src/mapping/visual_point.cc)
target_link_libraries(livo_atlas_vio PUBLIC livo_atlas_camera livo_atlas_lio
  pcl_filters opencv_imgproc Sophus::Sophus)
if(LIVO_ATLAS_USE_OPENMP AND OpenMP_CXX_FOUND)
  include(ProcessorCount)
  ProcessorCount(livo_atlas_processor_count)
  if(livo_atlas_processor_count GREATER 4)
    set(livo_atlas_processor_count 4)
  endif()
  foreach(target livo_atlas_lio livo_atlas_vio)
    target_link_libraries(${target} PUBLIC OpenMP::OpenMP_CXX)
    if(livo_atlas_processor_count GREATER 1)
      target_compile_definitions(${target} PRIVATE
        MP_EN MP_PROC_NUM=${livo_atlas_processor_count})
    endif()
  endforeach()
endif()
livo_atlas_library(frontend src/frontend/frontend.cc
  src/frontend/vision/image_options.cc src/frontend/imu/imu_propagation.cc)
target_link_libraries(livo_atlas_frontend PUBLIC livo_atlas_calibration
  livo_atlas_vio livo_atlas_lio livo_atlas_imu pcl_filters)
livo_atlas_library(colorizer src/mapping/cloud_colorizer.cc)
target_link_libraries(livo_atlas_colorizer PUBLIC livo_atlas_camera pcl_common)
livo_atlas_library(io src/io/livo_exporter.cc)
target_link_libraries(livo_atlas_io PUBLIC livo_atlas_camera Eigen3::Eigen
  pcl_io pcl_filters opencv_core opencv_imgcodecs)
livo_atlas_library(core src/core/livo_system.cc src/core/system_options.cc src/core/livo_config.cc)
target_link_libraries(livo_atlas_core PUBLIC livo_atlas_frontend
  livo_atlas_synchronization livo_atlas_preprocess livo_atlas_colorizer livo_atlas_io livo_atlas_keyframes)

install(TARGETS livo_atlas_build_options livo_atlas_calibration livo_atlas_camera livo_atlas_keyframes
  livo_atlas_synchronization livo_atlas_preprocess livo_atlas_imu livo_atlas_lio
  livo_atlas_vio livo_atlas_frontend livo_atlas_colorizer livo_atlas_io livo_atlas_core
  EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  PATTERN ros1 EXCLUDE PATTERN ros2 EXCLUDE PATTERN livox_ros_driver EXCLUDE)

if(LIVO_ATLAS_ROS_VERSION STREQUAL "0")
  include(CMakePackageConfigHelpers)
  set(livo_atlas_config_dir ${CMAKE_INSTALL_LIBDIR}/cmake/livo_atlas)
  set(LIVO_ATLAS_CONFIG_OPENMP FALSE)
  if(LIVO_ATLAS_USE_OPENMP AND OpenMP_CXX_FOUND)
    set(LIVO_ATLAS_CONFIG_OPENMP TRUE)
  endif()
  configure_package_config_file(cmake/livo_atlasConfig.cmake.in
    ${PROJECT_BINARY_DIR}/livo_atlasConfig.cmake
    INSTALL_DESTINATION ${livo_atlas_config_dir})
  write_basic_package_version_file(${PROJECT_BINARY_DIR}/livo_atlasConfigVersion.cmake
    VERSION ${PROJECT_VERSION} COMPATIBILITY SameMajorVersion)
  install(EXPORT livo_atlasTargets NAMESPACE livo_atlas::
    DESTINATION ${livo_atlas_config_dir})
  install(FILES ${PROJECT_BINARY_DIR}/livo_atlasConfig.cmake
    ${PROJECT_BINARY_DIR}/livo_atlasConfigVersion.cmake cmake/sophus_target.cmake
    DESTINATION ${livo_atlas_config_dir})
endif()
