livo_atlas_library(loop_backend src/backend/loop_admission.cc
  src/backend/loop_selection.cc)
target_include_directories(livo_atlas_loop_backend PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_loop_backend PUBLIC livo_atlas_backend
  livo_atlas_loop_closure)
install(TARGETS livo_atlas_loop_backend EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
