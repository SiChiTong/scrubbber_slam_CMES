add_subdirectory(msg)

add_library(${PROJECT_NAME}.common
        keyframe.cc
        trajectory.cc
        )

# make sure messages are compiled first
set(mapping_msgs
        ivlocmsg_generate_messages
        ivactuator_generate_messages
        ivapp_generate_messages
        avos_new_generate_messages
        avos_x_generate_messages
        cloud_msgs_generate_messages
        common_msgs_generate_messages
        ivlocmsg_generate_messages
        monitor_msgs_generate_messages
        new_ivapp_generate_messages
        x_ivapp_generate_messages
        new_ivsensorgps_generate_messages
        new_velodyne_msgs_generate_messages
        velodyne_msgs_generate_messages
        mainstream_msgs_generate_messages
        lads_generate_messages
        lads_velodyne_msgs_generate_messages
        #suteng_irad_generate_messages
        )

add_dependencies(${PROJECT_NAME}.common ${mapping_msgs})

target_link_libraries(${PROJECT_NAME}.common
        ${PCL_LIBRARIES})