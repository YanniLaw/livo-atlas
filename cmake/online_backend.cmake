livo_atlas_library(online_backend src/backend/loop_task.cc
  src/backend/loop_worker.cc)
target_link_libraries(livo_atlas_online_backend PUBLIC livo_atlas_loop_backend
  livo_atlas_io)
install(TARGETS livo_atlas_online_backend EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
