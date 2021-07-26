#!/bin/bash
# g2o
tn="-j"
echo "编译使用参数：$tn"
cd thirdparty/g2o-fixed
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# yaml-cpp
cd ../../yaml-cpp-fixed
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# xml-cpp
cd ../../xml-cpp-fixed
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# conf-cc
cd ../../conf_fixed
cd test
./compile.sh
cd ..
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# points_height
cd ../../point_height
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# oss
cd ../../aliyun-oss-cpp-sdk
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# hesai lidar
cd ../../hesai
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# suteng lidar
cd ../../suteng
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make $tn

# jsoncpp
cd ../../jsoncpp
#mkdir build
cd build
#cmake .. -DCMAKE_BUILD_TYPE=Release
#make $tn
cd ../../../

pwd

# main project
mkdir build
cd build
cmake .. -DENABLE_TESTS=OFF -DCMAKE_BUILD_TYPE=Release
make $tn data_fetching check_in preprocessing dr_frontend lidar_frontend run_opti_s1 run_opti_s2 run_loop_detect run_loop_compute run_checkout merge_db update_db_id merge_map

cd ..
