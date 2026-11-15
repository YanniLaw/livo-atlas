livo_atlas_library(online_gaussian src/gaussian/wire_protocol.cc
  src/gaussian/unix_socket.cc)
target_include_directories(livo_atlas_online_gaussian PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_online_gaussian PUBLIC opencv_core)
install(TARGETS livo_atlas_online_gaussian EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
