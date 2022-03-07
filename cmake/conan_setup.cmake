list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
  message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
  file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/0.17.0/conan.cmake"
       "${CMAKE_BINARY_DIR}/conan.cmake" TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

if((NOT EXISTS ${PROJECT_BINARY_DIR}/conaninfo.txt) OR (${PROJECT_SOURCE_DIR}/cmake/conan_setup.cmake IS_NEWER_THAN
                                                        ${PROJECT_BINARY_DIR}/conaninfo.txt))
  conan_cmake_configure(
    REQUIRES
    catch2/2.13.8
    spdlog/1.9.2
    GENERATORS
    cmake_find_package_multi
    OPTIONS
    catch2:with_main=True)

  foreach(TYPE ${CMAKE_CONFIGURATION_TYPES})
    conan_cmake_autodetect(conan_settings BUILD_TYPE ${TYPE})
    conan_cmake_install(
      PATH_OR_REFERENCE
      .
      BUILD
      missing
      SETTINGS
      ${conan_settings})
  endforeach()
endif()
