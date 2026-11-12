livo_atlas_library(loop_backend src/backend/loop_admission.cc
  src/backend/loop_selection.cc)
target_include_directories(livo_atlas_loop_backend PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_loop_backend PUBLIC livo_atlas_backend
  livo_atlas_loop_closure)
install(TARGETS livo_atlas_loop_backend EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

add_library(livo_atlas_loop_proposer STATIC src/io/loop_proposer.cc)
target_include_directories(livo_atlas_loop_proposer PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_loop_proposer PUBLIC livo_atlas_loop_backend
  livo_atlas_loop_io)
add_executable(livo_atlas_propose_loops tools/propose_loops.cc)
target_link_libraries(livo_atlas_propose_loops PRIVATE livo_atlas_loop_proposer)
set_target_properties(livo_atlas_propose_loops PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
if(UNIX AND NOT APPLE)
  set_target_properties(livo_atlas_propose_loops PROPERTIES
    INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
elseif(APPLE)
  set_target_properties(livo_atlas_propose_loops PROPERTIES
    INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}")
endif()
install(TARGETS livo_atlas_propose_loops RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
