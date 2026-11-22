set(livo_atlas_gaussian_cache_type "")
# Honor normal variables set by an enclosing CMake project.
if(NOT DEFINED LIVO_ATLAS_WITH_ONLINE_GAUSSIAN OR
    (DEFINED CACHE{LIVO_ATLAS_WITH_ONLINE_GAUSSIAN} AND
     "${LIVO_ATLAS_WITH_ONLINE_GAUSSIAN}" STREQUAL
     "$CACHE{LIVO_ATLAS_WITH_ONLINE_GAUSSIAN}"))
  get_property(livo_atlas_gaussian_cache_type CACHE LIVO_ATLAS_WITH_ONLINE_GAUSSIAN
    PROPERTY TYPE)
  set(LIVO_ATLAS_WITH_ONLINE_GAUSSIAN AUTO CACHE STRING
    "Build the online Gaussian bridge (AUTO, ON or OFF)")
  set_property(CACHE LIVO_ATLAS_WITH_ONLINE_GAUSSIAN PROPERTY TYPE STRING)
  set_property(CACHE LIVO_ATLAS_WITH_ONLINE_GAUSSIAN PROPERTY STRINGS AUTO ON OFF)
endif()

string(TOUPPER "${LIVO_ATLAS_WITH_ONLINE_GAUSSIAN}" livo_atlas_gaussian_mode)
if(livo_atlas_gaussian_cache_type STREQUAL "BOOL" AND
    NOT livo_atlas_gaussian_mode STREQUAL "AUTO")
  # Keep the effective value of caches created by the former option().
  if(LIVO_ATLAS_WITH_ONLINE_GAUSSIAN)
    set(livo_atlas_gaussian_mode ON)
  else()
    set(livo_atlas_gaussian_mode OFF)
  endif()
  set_property(CACHE LIVO_ATLAS_WITH_ONLINE_GAUSSIAN PROPERTY VALUE
    "${livo_atlas_gaussian_mode}")
endif()

set(livo_atlas_build_online_gaussian OFF)
if(livo_atlas_gaussian_mode STREQUAL "AUTO")
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(livo_atlas_build_online_gaussian ON)
  endif()
elseif(livo_atlas_gaussian_mode MATCHES "^(ON|TRUE|YES|Y|1)$")
  set(livo_atlas_build_online_gaussian ON)
elseif(NOT livo_atlas_gaussian_mode MATCHES "^(OFF|FALSE|NO|N|0)$")
  message(FATAL_ERROR "LIVO_ATLAS_WITH_ONLINE_GAUSSIAN must be AUTO, ON or OFF")
endif()
if(livo_atlas_build_online_gaussian AND NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "LIVO_ATLAS_WITH_ONLINE_GAUSSIAN=ON requires Linux")
endif()
message(STATUS "Online Gaussian bridge ${livo_atlas_build_online_gaussian}"
  " (${livo_atlas_gaussian_mode})")
