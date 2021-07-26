add_library(${PROJECT_NAME}.DR
#        dr.cc
#        dr_impl.cc
#        ml_dr.cc
        dr_pre_integration.cc
        )

add_dependencies(${PROJECT_NAME}.DR
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )

target_link_libraries(${PROJECT_NAME}.DR
        ${PROJECT_NAME}.common
        ${PROJECT_NAME}.tools
        )
