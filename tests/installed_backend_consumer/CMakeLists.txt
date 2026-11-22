cmake_minimum_required(VERSION 3.16)
project(livo_atlas_backend_consumer LANGUAGES CXX)

find_package(livo_atlas CONFIG REQUIRED)
add_executable(backend_consumer main.cc)
target_compile_features(backend_consumer PRIVATE cxx_std_17)
target_link_libraries(backend_consumer PRIVATE livo_atlas::backend)

enable_testing()
add_test(NAME backend_consumer COMMAND backend_consumer)
