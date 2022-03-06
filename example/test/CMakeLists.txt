include(CTest)

add_test(NAME cli.runs_planning COMMAND ${PROJECT_NAME})
set_tests_properties(cli.runs_planning PROPERTIES PASS_REGULAR_EXPRESSION "Planning")

add_test(NAME cli.version_matches COMMAND ${PROJECT_NAME})
set_tests_properties(cli.version_matches PROPERTIES PASS_REGULAR_EXPRESSION "Test application ${PROJECT_VERSION}")
