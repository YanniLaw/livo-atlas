find_package(PCL REQUIRED COMPONENTS common filters io registration kdtree)

livo_atlas_library(loop_closure src/backend/loop_closure.cc)
target_include_directories(livo_atlas_loop_closure PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_loop_closure PUBLIC livo_atlas_geometry
  PRIVATE pcl_registration pcl_kdtree)
install(TARGETS livo_atlas_loop_closure EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
