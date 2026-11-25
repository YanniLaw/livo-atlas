# Record the versions resolved for this build, without inspecting system headers.
set(livo_atlas_build_eigen_version "${Eigen3_VERSION}")
if(NOT livo_atlas_build_eigen_version)
  set(livo_atlas_build_eigen_version "${EIGEN3_VERSION_STRING}")
endif()
set(livo_atlas_build_gtsam_version "${GTSAM_VERSION}")
if(NOT livo_atlas_build_gtsam_version)
  set(livo_atlas_build_gtsam_version "${GTSAM_VERSION_STRING}")
endif()
configure_file(${CMAKE_CURRENT_LIST_DIR}/build_info.txt.in
  ${PROJECT_BINARY_DIR}/livo_atlas_build_info.txt @ONLY)
install(FILES ${PROJECT_BINARY_DIR}/livo_atlas_build_info.txt
  DESTINATION ${CMAKE_INSTALL_DATADIR}/livo_atlas)
