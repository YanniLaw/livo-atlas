livo_atlas_library(online_backend src/core/loop_backend.cc src/core/loop_commit.cc src/backend/loop_task.cc
  src/backend/loop_worker.cc src/io/loop_closure_store.cc
  src/io/loop_admission_io.cc src/io/loop_options.cc src/io/loop_diagnostics.cc
  third_party/cartographer/cartographer/common/task.cc
  third_party/cartographer/cartographer/common/thread_pool.cc)
target_include_directories(livo_atlas_online_backend PRIVATE ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/third_party/cartographer)
target_link_libraries(livo_atlas_online_backend PUBLIC livo_atlas_loop_backend
  livo_atlas_io livo_atlas_core)
install(TARGETS livo_atlas_online_backend EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(FILES third_party/cartographer/LICENSE
  DESTINATION ${CMAKE_INSTALL_DATADIR}/livo_atlas/licenses/cartographer)

add_executable(livo_atlas_check_loops tools/check_loops.cc)
target_include_directories(livo_atlas_check_loops PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_check_loops PRIVATE livo_atlas::online_backend)
set_target_properties(livo_atlas_check_loops PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
if(UNIX AND NOT APPLE)
  set_target_properties(livo_atlas_check_loops PROPERTIES INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
elseif(APPLE)
  set_target_properties(livo_atlas_check_loops PROPERTIES INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}")
endif()
install(TARGETS livo_atlas_check_loops RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
