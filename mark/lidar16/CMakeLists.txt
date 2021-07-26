add_library(${PROJECT_NAME}.lidar16

        ##lego-loam
        lidar_matching/feature_matching/local_mapper.cc
        lidar_matching/feature_matching/matching_param.cc
        ##lego-loam
        lidar_matching/lego_loam/image_projection.cc
        lidar_matching/lego_loam/feature_association.cc
#        lidar_matching/lego_loam/local_map.cc

#        lidar_matching/mapping_point_types.h

        ##map saver
        map_saver/map_saver.cc

        lego_loam_interface.cc
        occupancy_mapping.cc
        ml_submap.cc
        ml_tracking.cc
        ml_frame.cc

        #回环检测
        loop_closing/ml_loop_closing.cc
#        loop_closing/mt_search/multi_thread_search.cc
#        loop_closing/resolution_matching/matcher/multi_resolution_map.cc
#        loop_closing/resolution_matching/matcher/multi_resolution_matcher.cc
#        loop_closing/resolution_matching/matcher/score_board.cc
#        loop_closing/resolution_matching/map/resolution_map.cc
        )

target_link_libraries(${PROJECT_NAME}.lidar16
        ${PROJECT_NAME}.common
        ${third_party_libs}
        )