add_library(project_options INTERFACE)

option(WARNINGS_AS_ERRORS "Treat compiler warings as errors" ON)

if(WARNINGS_AS_ERRORS)
  if(MSVC)
    set(PROJECT_WARNINGS_CXX /WX)
  else()
    set(PROJECT_WARNINGS_CXX -Werror)
  endif()

  target_compile_options(project_options INTERFACE $<$<COMPILE_LANGUAGE:CXX>:${PROJECT_WARNINGS_CXX}>)
endif()
target_compile_features(project_options INTERFACE cxx_std_${CMAKE_CXX_STANDARD})
