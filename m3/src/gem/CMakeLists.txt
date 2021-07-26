add_executable(data_fetching run_data_fetching.cc)
target_link_libraries(data_fetching
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(check_in run_check_in.cc)
target_link_libraries(check_in
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(preprocessing run_preprocessing.cc)
target_link_libraries(preprocessing
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(dr_frontend run_dr_frontend.cc)
target_link_libraries(dr_frontend
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(lidar_frontend run_lidar_frontend.cc)
target_link_libraries(lidar_frontend
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(run_opti_s1 run_opti_s1.cc)
target_link_libraries(run_opti_s1
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(run_loop_detect run_loop_detect.cc)
target_link_libraries(run_loop_detect
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(run_loop_compute run_loop_compute.cc)
target_link_libraries(run_loop_compute
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(run_opti_s2 run_opti_s2.cc)
target_link_libraries(run_opti_s2
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )

add_executable(run_checkout run_checkout.cc)
target_link_libraries(run_checkout
        ${PROJECT_NAME}.pipeline
        ${PROJECT_NAME}.io
        )
