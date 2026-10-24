set(livo_atlas_runtime_libraries ${CMAKE_THREAD_LIBS_INIT})
if(LIVO_ATLAS_USE_OPENMP AND OpenMP_CXX_FOUND)
  list(APPEND livo_atlas_runtime_libraries ${OpenMP_CXX_LIBRARIES})
endif()
# External catkin projects receive library files, not the original targets.
# Keep dependents before dependencies for static linking as well.
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES livo_atlas_runtime livo_atlas_frontend livo_atlas_output livo_atlas_io
    livo_atlas_vio livo_atlas_lio livo_atlas_imu livo_atlas_synchronization
    livo_atlas_preprocess livo_atlas_camera livo_atlas_calibration
    ${livo_atlas_runtime_libraries}
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs roscpp rospy std_msgs
    pcl_ros tf message_runtime cv_bridge image_transport visualization_msgs
  DEPENDS PCL OpenCV Sophus)

add_library(livo_atlas_ros1_conversions src/ros1/measurement_conversions.cc)
target_include_directories(livo_atlas_ros1_conversions PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(livo_atlas_ros1_conversions PUBLIC livo_atlas::core
  ${catkin_LIBRARIES})
add_library(livo_atlas_ros1_mapper src/ros1/mapping_adapter.cc src/ros1/parameters.cc src/ros1/result_publisher.cc)
target_link_libraries(livo_atlas_ros1_mapper PUBLIC livo_atlas::runtime livo_atlas_ros1_conversions)
set_target_properties(livo_atlas_ros1_mapper livo_atlas_ros1_conversions PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
if(UNIX AND NOT APPLE)
  set_target_properties(livo_atlas_ros1_mapper livo_atlas_ros1_conversions PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()
install(TARGETS livo_atlas_ros1_mapper livo_atlas_ros1_conversions
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY include/livo_atlas/ros1
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/livo_atlas)
install(DIRECTORY include/livox_ros_driver
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
