find_package(PCL REQUIRED COMPONENTS common filters io registration kdtree)

livo_atlas_library(loop_closure src/backend/loop_closure.cc
  src/backend/loop_registration.cc src/backend/loop_quality.cc)
target_include_directories(livo_atlas_loop_closure PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_loop_closure PUBLIC livo_atlas_geometry
  PRIVATE pcl_registration pcl_kdtree)
install(TARGETS livo_atlas_loop_closure EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

# Keep dataset orchestration private to the tool and its integration tests.
add_library(livo_atlas_loop_io STATIC src/io/loop_verifier.cc)
target_include_directories(livo_atlas_loop_io PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_loop_io PUBLIC livo_atlas_loop_closure livo_atlas_io)
add_executable(livo_atlas_verify_loops tools/verify_loops.cc)
target_link_libraries(livo_atlas_verify_loops PRIVATE livo_atlas_loop_io)
set_target_properties(livo_atlas_verify_loops PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
if(UNIX AND NOT APPLE)
  set_target_properties(livo_atlas_verify_loops PROPERTIES
    INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")
elseif(APPLE)
  set_target_properties(livo_atlas_verify_loops PROPERTIES
    INSTALL_RPATH "@loader_path/../${CMAKE_INSTALL_LIBDIR}")
endif()
install(TARGETS livo_atlas_verify_loops RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
