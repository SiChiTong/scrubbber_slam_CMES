rm -rf install
mkdir install
mkdir -p install/libs install/config
mkdir -p install/scripts

cp ./bin/mapping ./install/
cp ./bin/mapping-ui ./install/
cp ./bin/debug-ui ./install/
cp ./build/src/core/libmapping.core.so ./install/libs/
cp ./build/src/common/libmapping.common.so ./install/libs/
cp ./build/src/io/libmapping.io.so ./install/libs/
cp ./build/src/tools/libmapping.tools.so ./install/libs/
cp ./build/src/pipeline/libmapping.pipeline.so ./install/libs/

cp ./config/mapping.yaml ./install/config/
cp ./config/server.yaml ./install/config/
cp ./thirdparty/conf_fixed/lib/libconf_cc.so ./install/libs    
cp ./thirdparty/suteng/lib/libsu_teng_driver.so ./install/libs
cp ./thirdparty/hesai/lib/libhesai_driver.so ./install/libs
cp ./thirdparty/g2o-fixed/lib/libg2o_* ./install/libs 
cp ./thirdparty/point_height/lib/libpoint_height.so ./install/libs
cp ./thirdparty/xml-cpp-fixed/lib/libxml_cpp.so ./install/libs 
cp ./thirdparty/yaml-cpp-fixed/build/libyaml-cpp.so.0.6.2 ./install/libs/libyaml-cpp.so.0.6
cp ./thirdparty/aliyun-oss-cpp-sdk/build/lib/libalibabacloud-oss-cpp-sdk.a ./install/libs/
cp ./thirdparty/suteng_IRAD/build/devel/lib/libsu_teng_driver_IRAD.so ./install/libs/

cp ./scripts/*.pyc ./install/scripts/
cp ./scripts/*.py ./install/scripts/

cp setup.bash install/

tar -czvf install_mapping.tar.gz install/
# rm -rf install
