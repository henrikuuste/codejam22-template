cmake_minimum_required(VERSION 3.15)
project(
  pathplanning
  VERSION 0.1.0
  LANGUAGES CXX)

if(CONAN_EXPORTED)
  list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../cmake)
  include(project_options)
  include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  conan_basic_setup()
endif()

find_package(spdlog REQUIRED)

set(LIB_SOURCES pathplanning.cpp)

if(BUILD_SHARED_LIBS)
  add_library(${PROJECT_NAME} SHARED ${LIB_SOURCES})
else()
  add_library(${PROJECT_NAME} STATIC ${LIB_SOURCES})
endif()
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PRIVATE project_options project_warnings)
target_link_libraries(${PROJECT_NAME} PUBLIC spdlog::spdlog)

include(GenerateExportHeader)
generate_export_header(${PROJECT_NAME} EXPORT_FILE_NAME include/${PROJECT_NAME}/export.h)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>)

include(GNUInstallDirs)
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include/${PROJECT_NAME} DESTINATION include)
