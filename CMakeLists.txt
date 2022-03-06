cmake_minimum_required(VERSION 3.15)
project(codejam22-dev LANGUAGES CXX)

enable_testing()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(conan_setup)

include(project_options)

add_subdirectory(lib)
add_subdirectory(example)
add_subdirectory(test)
