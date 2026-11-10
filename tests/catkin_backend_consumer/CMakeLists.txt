cmake_minimum_required(VERSION 3.16)
project(livo_atlas_catkin_backend_consumer LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS livo_atlas)
add_executable(catkin_backend_consumer ../installed_backend_consumer/main.cc)
target_compile_features(catkin_backend_consumer PRIVATE cxx_std_17)
target_include_directories(catkin_backend_consumer PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(catkin_backend_consumer PRIVATE ${catkin_LIBRARIES})
if(EXPECT_BACKEND)
  target_compile_definitions(catkin_backend_consumer PRIVATE EXPECT_BACKEND)
endif()

enable_testing()
add_test(NAME catkin_backend_consumer COMMAND catkin_backend_consumer)
