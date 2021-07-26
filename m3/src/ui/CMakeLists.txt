# init_qt: Let's do the CMake job for us
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

find_package(Qt5 REQUIRED Widgets Concurrent)
find_package(VTK 6 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR}/build/src/ui)
# include_directories(${PROJECT_SOURCE_DIR}/src/ui/debug-ui/)

include_directories(${VTK_INCLUDE_DIRS})

# mapping-ui 现场建图界面
add_executable(mapping-ui mapping-ui/main.cpp
        mapping-ui/mainwindow.cpp
        mapping-ui/logindialog.cpp)

target_link_libraries(mapping-ui
        Qt5::Widgets
        Qt5::Concurrent
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.tools
        ${PROJECT_NAME}.core
        ${third_party_libs})

# debug-ui 调试界面
# add_executable(debug-ui debug-ui/main.cpp debug-ui/debug_ui_mainwindow.cpp debug-ui/debug_ui_mainwindow.ui)

# set(vtk_libs
#         vtklz4-8.2
#         )

# target_link_libraries(debug-ui
#         PRIVATE
#         Qt5::Widgets
#         ${PROJECT_NAME}.pipeline
#         ${PROJECT_NAME}.io
#         ${PROJECT_NAME}.tools
#         ${VTK_LIBRARIES}

#         ${vtk_libs}

#         ${third_party_libs})
