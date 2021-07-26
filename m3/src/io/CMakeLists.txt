add_library(${PROJECT_NAME}.io
        yaml_io.cc
        xml_io.cc
        db_io.cc
        oss_io.cc
        file_io.cc
        db_rw.cc
        sqlite3.c
        )

target_link_libraries(${PROJECT_NAME}.io
        ${third_party_libs}
        ${PROJECT_NAME}.common
        )

add_dependencies(${PROJECT_NAME}.io
        ${PROJECT_NAME}.common
        )