cmake_minimum_required(VERSION 3.5)
project(mapping)

set(DEFAULT_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-Wall -Wno-reorder -Wno-unused-function -fPIC -no-pie -Wno-sign-compare -Wno-unused-result")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
include(cmake/packages.cmake)

option(ENABLE_TESTS "Enable tests" OFF)
option(ENABLE_GEM "Enable gem" OFF)

if (${ENABLE_TESTS})
    enable_testing()
    add_subdirectory(test)
endif ()

add_subdirectory(src)
