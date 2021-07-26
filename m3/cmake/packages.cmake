list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# catkin
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        sensor_msgs
        pcl_ros
        pcl_conversions
        )
include_directories(${catkin_INCLUDE_DIRS})
# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# PCL
find_package(PCL REQUIRED Components common io filters visualization)
include_directories(${PCL_INCLUDE_DIRS})
# cholmod
find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})
# opencv
find_package(OpenCV 4 QUIET NO_MODULE)
if (NOT OpenCV_FOUND)
    set(OpenCV_LIBS "")
    set(OpenCV_INCLUDE_DIRS "")
    set(OpenCV_DIR /usr/local/opencv3/share/OpenCV)
    find_package(OpenCV 3 REQUIRED)
endif ()

include_directories(${OpenCV_INCLUDE_DIRS})
# boost
find_package(Boost REQUIRED COMPONENTS system)
# glog
find_package(Glog REQUIRED COMPONENTS system)

# flags
add_definitions(${PCL_DEFINITIONS})

# omp
find_package(OpenMP)
if (OPENMP_FOUND)
    message("Use OpenMP features")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${OpenMP_CXX_FLAGS}")
endif ()

# curl and openssl
include(FindCURL)
include(FindOpenSSL)

# Qt
# find_package(Qt5 COMPONENTS Widgets REQUIRED)

# VTK
# find_package(VTK 8 REQUIRED)

# thirdparty
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/yaml-cpp-fixed/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/xml-cpp-fixed/src)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o-fixed/ ${PROJECT_SOURCE_DIR}/thirdparty/g2o-fixed/build)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/velodyne/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/point_height/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/aliyun-oss-cpp-sdk/sdk/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/jsoncpp/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/conf_fixed/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/conf_fixed/test)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/Sophus/)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/hesai/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/suteng/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/suteng/src)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/suteng_IRAD/include)
include_directories(${PROJECT_SOURCE_DIR}/thirdparty/suteng_IRAD)

set(third_party_libs
        ${OpenCV_LIBS}
        ${BOOST_LIBRARIES}
        ${PCL_LIBRARIES}
        glog gflags
        ${CURL_LIBRARIES}
        ${OPENSSL_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/thirdparty/yaml-cpp-fixed/build/libyaml-cpp.so
        ${PROJECT_SOURCE_DIR}/thirdparty/xml-cpp-fixed/lib/libxml_cpp.so
        ${PROJECT_SOURCE_DIR}/thirdparty/conf_fixed/lib/libconf_cc.so
        ${PROJECT_SOURCE_DIR}/thirdparty/point_height/lib/libpoint_height.so
        pthread dl
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o-fixed/lib/libg2o_core.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o-fixed/lib/libg2o_stuff.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o-fixed/lib/libg2o_types_slam3d.so
        ${PROJECT_SOURCE_DIR}/thirdparty/g2o-fixed/lib/libg2o_types_slam2d.so
        ${PROJECT_SOURCE_DIR}/thirdparty/aliyun-oss-cpp-sdk/build/lib/libalibabacloud-oss-cpp-sdk.a
        ${PROJECT_SOURCE_DIR}/thirdparty/jsoncpp/build/src/lib_json/libjsoncpp.a
        ${CHOLMOD_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/thirdparty/hesai/lib/libhesai_driver.so
        ${catkin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/thirdparty/suteng/lib/libsu_teng_driver.so
        ${PROJECT_SOURCE_DIR}/thirdparty/suteng_IRAD/build/devel/lib/libsu_teng_driver_IRAD.so
        )

if (${ENABLE_TESTS})
    set(third_party_libs ${third_party_libs} gtest)
endif ()