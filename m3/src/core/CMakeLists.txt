add_library(${PROJECT_NAME}.core
        dead_reckoning/dr.cc
        lidar_matching/feature_matching/range_image_projection.cc
        lidar_matching/feature_matching/feature_extractor.cc
        lidar_matching/feature_matching/feature_tracker.cc
        lidar_matching/feature_matching/local_mapper.cc
        lidar_matching/feature_matching/feature_matching.cc
        lidar_matching/feature_matching/matching_param.cc
        lidar_matching/ndt_matching/ndt_matching.cc

        lidar_matching/lego_loam/lego_loam_interface.cc
        lidar_matching/lego_loam/lego_loam_param.cc
        lidar_matching/lego_loam/feature_association.cc
        lidar_matching/lego_loam/image_projection.cc
        lidar_matching/lego_loam/local_map.cc

        coordinate_transform/gps_trans.cc
        ndt_omp/src/pclomp/gicp_omp.cpp
        ndt_omp/src/pclomp/ndt_omp.cpp
        ndt_omp/src/pclomp/voxel_grid_covariance_omp.cpp
        resolution_matching/map/resolution_map.cc
        resolution_matching/matcher/multi_resolution_map.cc
        resolution_matching/matcher/multi_resolution_matcher.cc
        resolution_matching/matcher/score_board.cc
        mem_control/mem_control.cc
        scan_context/scan_context.cc
        mt_search/multi_thread_search.cc
        )

target_link_libraries(${PROJECT_NAME}.core
        ${PROJECT_NAME}.io
        ${PCL_LIBRARIES}
        )


add_dependencies(${PROJECT_NAME}.core ${PROJECT_NAME}.common ${PROJECT_NAME}.io)