#!/bin/bash
# g2o
if [ $# -ge 1 ]; then
	tn="$1"
else
	tn="-j2"
fi 
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

# suteng lidar
cd ../../suteng_IRAD
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
cmake .. -DENABLE_TESTS=OFF -DCATKIN_ENABLE_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DEABLE_GEM=OFF
make $tn mapping

cd ..

