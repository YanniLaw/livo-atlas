cmake_minimum_required(VERSION 3.16)
project(livo_atlas_3dgs_probe LANGUAGES C CXX)

find_package(livo_atlas REQUIRED CONFIG)
include(CTest)

add_executable(livo_atlas_prepare_3dgs_views prepare_views.cc)
target_compile_features(livo_atlas_prepare_3dgs_views PRIVATE cxx_std_17)
target_link_libraries(livo_atlas_prepare_3dgs_views PRIVATE livo_atlas::io)

if(BUILD_TESTING)
  add_executable(p603b_make_fixture ../../tests/3dgs_probe/make_fixture.cc)
  target_compile_features(p603b_make_fixture PRIVATE cxx_std_17)
  target_link_libraries(p603b_make_fixture PRIVATE livo_atlas::io)
endif()
