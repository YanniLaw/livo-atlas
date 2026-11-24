find_package(GTSAM 4.2 CONFIG REQUIRED)

livo_atlas_library(backend src/optimization/optimizer.cc)
target_include_directories(livo_atlas_backend PRIVATE ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(livo_atlas_backend PUBLIC livo_atlas_keyframes PRIVATE gtsam)

install(TARGETS livo_atlas_backend EXPORT livo_atlasTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
