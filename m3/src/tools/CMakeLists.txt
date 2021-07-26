add_library(${PROJECT_NAME}.tools
        pointcloud_convert/packets_parser.cc
        pointcloud_convert/pc_convertor.cc
        pointcloud_convert/compensator.cc
        pointcloud_convert/suteng_pc_convertor.cc
        pointcloud_convert/suteng_irad_pc_convertor.cc
        pointcloud_convert/hesai_pc_convertor.cc
        perception_interface/interface.cc
        perception_interface/pcl_compensator.cc
        # intensity_calib/io.cc
        # intensity_calib/intensity_calibrator.cc
        # pcd_to_png/pcd_to_png.cc
        # save_file/save_file.cc
        gmm_map_generator/idp_gmm_map_generator.cc
        gmm_map_generator/lads_gmm_map_generator.cc
        gmm_map_generator/tiles_gmm_map_generator.cc
        gmm_map_generator/gmm_map_io.cc
        map_splitter/map_splitter.cc
        )

target_link_libraries(${PROJECT_NAME}.tools
        ${PROJECT_NAME}.io
        ${PCL_LIBRARIES}
        ${third_party_libs}
        )

add_dependencies(${PROJECT_NAME}.tools ${PROJECT_NAME}.common ${PROJECT_NAME}.io)
