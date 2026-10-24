catkin_package(
  INCLUDE_DIRS include
  LIBRARIES livo_atlas_frontend livo_atlas_synchronization livo_atlas_preprocess
    livo_atlas_camera livo_atlas_output livo_atlas_io
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs roscpp rospy std_msgs
    pcl_ros tf message_runtime cv_bridge image_transport visualization_msgs
  DEPENDS PCL OpenCV Sophus)

add_library(livo_atlas_ros1_conversions src/ros1/measurement_conversions.cc)
target_include_directories(livo_atlas_ros1_conversions PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(livo_atlas_ros1_conversions PUBLIC livo_atlas::core
  ${catkin_LIBRARIES})
add_library(livo_atlas_ros1_output src/ros1/result_publisher.cc)
target_include_directories(livo_atlas_ros1_output PUBLIC
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(livo_atlas_ros1_output PUBLIC livo_atlas::output
  ${catkin_LIBRARIES})
add_library(livo_atlas_ros1_mapper src/LIVMapper.cpp src/ros1/calibration_reader.cc
  src/ros1/voxel_options_reader.cc)
target_link_libraries(livo_atlas_ros1_mapper PUBLIC livo_atlas::core
  livo_atlas::output livo_atlas::io livo_atlas_ros1_output
  livo_atlas_ros1_conversions)
add_executable(fastlivo_mapping src/main.cpp)
target_link_libraries(fastlivo_mapping PRIVATE livo_atlas_ros1_mapper)
set_target_properties(livo_atlas_ros1_mapper livo_atlas_ros1_conversions
  livo_atlas_ros1_output fastlivo_mapping PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
if(UNIX AND NOT APPLE)
  set_target_properties(livo_atlas_ros1_mapper livo_atlas_ros1_conversions
    livo_atlas_ros1_output PROPERTIES INSTALL_RPATH "$ORIGIN")
  set_target_properties(fastlivo_mapping PROPERTIES INSTALL_RPATH "$ORIGIN/..")
endif()
find_package(mimalloc QUIET)
if(mimalloc_FOUND)
  target_link_libraries(fastlivo_mapping PRIVATE mimalloc)
endif()
install(TARGETS fastlivo_mapping livo_atlas_ros1_mapper
  livo_atlas_ros1_conversions livo_atlas_ros1_output
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/livo_atlas/ros1
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livo_atlas)
install(DIRECTORY include/livox_ros_driver
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
install(FILES include/LIVMapper.h DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
