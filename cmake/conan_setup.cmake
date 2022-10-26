list(APPEND CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})
list(APPEND CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})

if(NOT EXISTS "${CMAKE_BINARY_DIR}/conan.cmake")
  message(STATUS "Downloading conan.cmake from https://github.com/conan-io/cmake-conan")
  file(DOWNLOAD "https://raw.githubusercontent.com/conan-io/cmake-conan/0.17.0/conan.cmake"
       "${CMAKE_BINARY_DIR}/conan.cmake" TLS_VERIFY ON)
endif()

include(${CMAKE_BINARY_DIR}/conan.cmake)

set(LIB_DEPS
    spdlog/1.9.2
    eigen/3.4.0
    units/2.3.1
    expected-lite/0.5.0
    magic_enum/0.8.1)
set(LIB_BUILD_DEPS)

if((NOT EXISTS ${PROJECT_BINARY_DIR}/conaninfo.txt) OR (${PROJECT_SOURCE_DIR}/cmake/conan_setup.cmake IS_NEWER_THAN
                                                        ${PROJECT_BINARY_DIR}/conaninfo.txt))
  conan_cmake_configure(
    REQUIRES
    ${LIB_DEPS}
    catch2/2.13.8
    docopt.cpp/0.6.3
    freeimage/3.18.0
    BUILD_REQUIRES
    ${LIB_BUILD_DEPS}
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
