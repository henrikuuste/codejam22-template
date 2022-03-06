cmake_minimum_required(VERSION 3.15)
project(
  pathplanning
  VERSION 0.1.0
  LANGUAGES CXX)

find_package(spdlog REQUIRED)

add_library(${PROJECT_NAME} STATIC pathplanning.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE project_options
  PUBLIC spdlog::spdlog)
