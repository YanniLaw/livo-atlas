livo_atlas_library(online_gaussian src/gaussian/wire_protocol.cc
  src/gaussian/unix_socket.cc src/gaussian/online_gaussian_mapper.cc
  src/gaussian/gaussian_map_index.cc src/gaussian/submap_protocol.cc
  src/io/gaussian_map_writer.cc src/io/gaussian_render_writer.cc src/gaussian/mapper_io.cc
  src/gaussian/submap_scheduler.cc src/gaussian/submap_commands.cc
  src/gaussian/render_protocol.cc src/gaussian/render_scheduler.cc
  src/gaussian/gaussian_render_output.cc
  src/gaussian/gaussian_render_snapshot.cc
  src/gaussian/render_validation.cc
  src/gaussian/render_metadata.cc
  src/gaussian/render_metadata_protocol.cc
  src/gaussian/holdout_split.cc
  src/gaussian/evaluation_protocol.cc
  src/gaussian/evaluation_scheduler.cc
  src/gaussian/gaussian_projection.cc
  src/gaussian/finalization.cc src/gaussian/gaussian_render.cc)
target_include_directories(livo_atlas_online_gaussian PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_online_gaussian PUBLIC livo_atlas_core
  opencv_core opencv_imgproc opencv_imgcodecs)
install(TARGETS livo_atlas_online_gaussian EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable(livo_atlas_select_gaussian_neighbor tools/select_gaussian_neighbor.cc)
target_include_directories(livo_atlas_select_gaussian_neighbor PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_select_gaussian_neighbor PRIVATE livo_atlas::online_gaussian)
set_target_properties(livo_atlas_select_gaussian_neighbor PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
  INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
install(TARGETS livo_atlas_select_gaussian_neighbor RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
