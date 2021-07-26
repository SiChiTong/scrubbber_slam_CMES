add_executable(mapping
        mapping_server/mapping_command.cc mapping_server/mapping_client.cc mapping_server/mapping_server.cc)

add_dependencies(mapping ${PROJECT_NAME}.common)

target_link_libraries(mapping
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.common
        ${third_party_libs})

add_executable(dump_db_data dump_db_data.cc)
target_link_libraries(dump_db_data
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(dump_and_merge_db_data dump_and_merge_db_data.cc)
target_link_libraries(dump_and_merge_db_data
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(pcd_intensity_remap pcd_intensity_remap.cc)
target_link_libraries(pcd_intensity_remap
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(merge_db merge_db.cc)
target_link_libraries(merge_db
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(update_db_id update_db_id.cc)
target_link_libraries(update_db_id
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_executable(sqlit_db_to_bin sqlit_db_to_bin.cc)
target_link_libraries(sqlit_db_to_bin
        ${PROJECT_NAME}.io
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )

add_subdirectory(quick_debug)