find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# Generic type support dlopens generated RMW libraries, even with static core.
set(livo_atlas_algorithm_shared ${BUILD_SHARED_LIBS})
set(BUILD_SHARED_LIBS ON)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/TrajectoryKeyframe.msg msg/KeyframeTrajectory.msg
  msg/GeometryMapSource.msg msg/GeometryMapParameters.msg msg/GeometryMap.msg
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs)
set(BUILD_SHARED_LIBS ${livo_atlas_algorithm_shared})
unset(livo_atlas_algorithm_shared)
rosidl_get_typesupport_target(livo_atlas_typesupport ${PROJECT_NAME}
  rosidl_typesupport_cpp)
ament_export_dependencies(rosidl_default_runtime)

add_library(livo_atlas_ros2_conversions src/ros2/measurement_conversions.cc)
target_link_libraries(livo_atlas_ros2_conversions livo_atlas::core)
ament_target_dependencies(livo_atlas_ros2_conversions
  sensor_msgs cv_bridge pcl_conversions livox_ros_driver2)
add_library(livo_atlas_ros2_node src/ros2/node_options.cc src/ros2/livo_node.cc
  src/ros2/livo_publisher.cc src/ros2/keyframe_trajectory_publisher.cc
  src/ros2/geometry_map_publisher.cc)
target_include_directories(livo_atlas_ros2_node PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_ros2_node livo_atlas_core
  livo_atlas_ros2_conversions "${livo_atlas_typesupport}")
ament_target_dependencies(livo_atlas_ros2_node rclcpp nav_msgs geometry_msgs
  visualization_msgs tf2_ros cv_bridge)
set_target_properties(livo_atlas_ros2_conversions
  livo_atlas_ros2_node PROPERTIES INSTALL_RPATH "$ORIGIN" INSTALL_RPATH_USE_LINK_PATH TRUE)
install(TARGETS livo_atlas_ros2_conversions livo_atlas_ros2_node
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
