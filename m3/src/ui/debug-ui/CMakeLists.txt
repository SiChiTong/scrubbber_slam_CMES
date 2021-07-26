cmake_minimum_required(VERSION 2.8.11)

project(debug-ui)

# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(VTK 8 REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(project_SOURCES main.cpp debug_ui_mainwindow.cpp)

add_executable(debug-ui ${project_SOURCES})
target_link_libraries(debug-ui ${PCL_LIBRARIES} Qt5::Widgets ${VTK_LIBRARIES} /usr/lib/x86_64-linux-gnu/libvtkGUISupportQt-7.1.so vtkGUISupportQt-7.1)
