find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_library(livo_atlas_ros2_conversions src/ros2/measurement_conversions.cc)
target_link_libraries(livo_atlas_ros2_conversions livo_atlas::core)
ament_target_dependencies(livo_atlas_ros2_conversions
  sensor_msgs cv_bridge pcl_conversions livox_ros_driver2)
add_library(livo_atlas_ros2_session src/ros2/mapping_session.cc)
target_include_directories(livo_atlas_ros2_session PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_ros2_session livo_atlas::core livo_atlas::output livo_atlas::io)
add_library(livo_atlas_ros2_mapper src/ros2/parameters.cc src/ros2/mapper_node.cc src/ros2/result_publisher.cc)
target_link_libraries(livo_atlas_ros2_mapper livo_atlas_ros2_session livo_atlas_ros2_conversions)
ament_target_dependencies(livo_atlas_ros2_mapper rclcpp nav_msgs geometry_msgs
  visualization_msgs tf2_ros cv_bridge)
set_target_properties(livo_atlas_ros2_conversions livo_atlas_ros2_session
  livo_atlas_ros2_mapper PROPERTIES INSTALL_RPATH "$ORIGIN" INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS livo_atlas_ros2_conversions livo_atlas_ros2_session livo_atlas_ros2_mapper
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
install(DIRECTORY include/livo_atlas/ros2 DESTINATION include/livo_atlas)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

set(LIVO_ATLAS_CONFIG_OPENMP FALSE)
if(LIVO_ATLAS_USE_OPENMP AND OpenMP_CXX_FOUND)
  set(LIVO_ATLAS_CONFIG_OPENMP TRUE)
endif()
configure_file(cmake/ros2_dependencies.cmake.in
  ${PROJECT_BINARY_DIR}/ros2_dependencies.cmake @ONLY)
install(FILES cmake/sophus_target.cmake DESTINATION share/${PROJECT_NAME}/cmake)
ament_export_targets(livo_atlasTargets HAS_LIBRARY_TARGET)
