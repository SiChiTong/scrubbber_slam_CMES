rm -rf install_onsite
mkdir install_onsite
mkdir -p install_onsite/libs install_onsite/config
mkdir -p install_onsite/scripts

cp ./bin/mapping-ui ./install_onsite/
cp ./build/src/core/libmapping.core.so ./install_onsite/libs/
cp ./build/src/common/libmapping.common.so ./install_onsite/libs/
cp ./build/src/io/libmapping.io.so ./install_onsite/libs/
cp ./build/src/tools/libmapping.tools.so ./install_onsite/libs/
cp ./build/src/pipeline/libmapping.pipeline.so ./install_onsite/libs/

cp ./config/mapping.yaml ./install_onsite/config/
cp ./config/server.yaml ./install_onsite/config/
cp ./thirdparty/conf_fixed/lib/libconf_cc.so ./install_onsite/libs 
cp ./thirdparty/g2o-fixed/lib/libg2o_* ./install_onsite/libs 
cp ./thirdparty/point_height/lib/libpoint_height.so ./install_onsite/libs
cp ./thirdparty/xml-cpp-fixed/lib/libxml_cpp.so ./install_onsite/libs 
cp ./thirdparty/yaml-cpp-fixed/build/libyaml-cpp.so.0.6.2 ./install_onsite/libs/libyaml-cpp.so.0.6
cp ./thirdparty/aliyun-oss-cpp-sdk/build/lib/libalibabacloud-oss-cpp-sdk.a ./install_onsite/libs/
cp ./thirdparty/build/lib/libalibabacloud-oss-cpp-sdk.a ./install_onsite/libs/
cp ./thirdparty/suteng/lib/libsu_teng_driver.so ./install_onsite/libs
cp ./thirdparty/suteng_IRAD/build/devel/lib/libsu_teng_driver_IRAD.so ./install_onsite/libs
cp ./thirdparty/hesai/lib/libhesai_driver.so ./install_onsite/libs

cp ./scripts/*.pyc ./install_onsite/scripts/

cp setup.bash install_onsite/

tar -czvf install_onsite_mapping.tar.gz install_onsite/
# rm -rf install
