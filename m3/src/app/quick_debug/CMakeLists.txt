cmake_minimum_required(VERSION 3.5)
project(quick_debug)

set(DEFAULT_BUILD_TYPE "Release")
#set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_COMPILER "g++")

set(CMAKE_CXX_FLAGS "-g -std=c++11 -Wall -Wno-reorder -fPIC -no-pie")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -std=c++11 -Wall -Wno-reorder")
set(CMAKE_CXX_FLAGS_DEBUG "-g -std=c++11 -Wall -Wno-reorder")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Qt
find_package(Qt5 REQUIRED Widgets Concurrent)

# OpenGL
#cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/src)

set(quick_debug_third_party_libs
        ${OPENGL_LIBRARIES}
        GLEW
        glog gflags
        )

add_subdirectory(src/algorithm)
add_subdirectory(src/platform)
add_subdirectory(src/renderGL)
add_subdirectory(src/map)

add_executable(quick_debug
        src/main.cpp
        src/quickdebugmainwindow.cpp
        src/qddisplaywidget.cc
        src/mapping_interface.cc
        src/map_data_loader.cc
        src/dialog.cpp
        src/optimizationdialog.cpp
        )

target_link_libraries(quick_debug
        Qt5::Widgets Qt5::Concurrent
        ${third_party_libs}
        ${quick_debug_third_party_libs}
        ${mapping_libs}

        ${PROJECT_NAME}.renderGL
        ${PROJECT_NAME}.algo
        ${PROJECT_NAME}.sys
        ${PROJECT_NAME}.map
        )