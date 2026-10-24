# Old Sophus exports variables; current distributions may export this target.
if(NOT TARGET Sophus::Sophus)
  if(NOT Sophus_LIBRARIES OR NOT Sophus_INCLUDE_DIRS)
    message(FATAL_ERROR "Sophus must provide its library and include directories")
  endif()
  add_library(Sophus::Sophus INTERFACE IMPORTED)
  set_target_properties(Sophus::Sophus PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${Sophus_INCLUDE_DIRS}"
    INTERFACE_LINK_LIBRARIES "${Sophus_LIBRARIES}")
endif()
