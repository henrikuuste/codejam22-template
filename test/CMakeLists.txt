include(CTest)

find_package(Catch2 REQUIRED)
include(Catch)

add_executable(core_test core_test.cpp)
target_link_libraries(core_test PUBLIC pathplanning::pathplanning Catch2::Catch2WithMain)
target_link_libraries(core_test PRIVATE project_options project_warnings)

catch_discover_tests(core_test)

add_executable(cost_test cost_test.cpp)
target_link_libraries(cost_test PUBLIC pathplanning::pathplanning Catch2::Catch2WithMain)
target_link_libraries(cost_test PRIVATE project_options project_warnings)

catch_discover_tests(cost_test)
