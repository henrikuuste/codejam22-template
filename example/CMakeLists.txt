cmake_minimum_required(VERSION 3.15)
project(
  codejam22-app
  VERSION 0.1.0
  LANGUAGES CXX)

add_executable(${PROJECT_NAME} app.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE pathplanning::pathplanning)

add_subdirectory(config)
add_subdirectory(test)
