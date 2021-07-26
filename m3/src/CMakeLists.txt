include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CATKIN_DEVEL_PREFIX}/include)

add_subdirectory(common)
add_subdirectory(core)
add_subdirectory(io)
add_subdirectory(tools)
add_subdirectory(pipeline)

set(mapping_libs
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.core
        ${PROJECT_NAME}.tools
        ${PROJECT_NAME}.pipeline
        )

#add_subdirectory(ui)
add_subdirectory(app)

if (${ENABLE_GEM})
    add_subdirectory(gem)
endif ()
