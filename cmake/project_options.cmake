set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)
FetchContent_Declare(_project_options
                     URL https://github.com/cpp-best-practices/project_options/archive/refs/tags/v0.18.1.zip)
FetchContent_MakeAvailable(_project_options)
include(${_project_options_SOURCE_DIR}/Index.cmake)
include(${_project_options_SOURCE_DIR}/src/DynamicProjectOptions.cmake)
set(WARNINGS_AS_ERRORS_DEFAULT OFF)
dynamic_project_options()
target_compile_features(project_options INTERFACE cxx_std_${CMAKE_CXX_STANDARD})
