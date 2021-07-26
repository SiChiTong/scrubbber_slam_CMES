#include <glog/logging.h>
#include <gtest/gtest.h>

#include "tools/lua_parser/configuration_file_resolver.h"
#include "tools/lua_parser/lua_parameter_dictionary.h"

#include "nodes/online/map_builder.h"
#include "interface.h"

#include "tools/point_height/point_height.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>


namespace mapping {

typedef pcl::PointXYZI PointXYZI;
typedef driver::velodyne::PointXYZRRIAR PointXYZRRIAR;
typedef pcl::PointCloud<PointXYZI> PointCloudXYZI;
typedef pcl::PointCloud<PointXYZRRIAR> PointCloudXYZRRIAR;

class InterTest : public ::testing::Test {
 protected:
  InterTest() {}

  ~InterTest() override {}
  void SetUp() override {
    auto file_resolver = make_unique<ConfigurationFileResolver>(
        std::vector<std::string>{CONFIGURATION_DIR});
    auto lua_parameter_dictionary = LuaParameterDictionary::NonReferenceCounted(
        kCode, std::move(file_resolver));

    interface = new Interface(lua_parameter_dictionary.get());
    map_builder = new MapBuilder(lua_parameter_dictionary.get());

  }

  void TearDown() override {}

  Interface* interface;
  MapBuilder *map_builder;

  float tolerance = 0.001;
};

TEST_F(InterTest, TestMethord_ProcessScan) {

  map_builder->Preprocessing();
    
  Frames* frames = &(map_builder->data_for_matching_[0].frames);
  std::vector<PacketsMsgPtr> three_packets;
  
  for(int i = 0; i < frames->size(); i++) {
      three_packets.push_back(frames->at(i).pre_packets_ptr);
      three_packets.push_back(frames->at(i).packets_ptr);
      three_packets.push_back(frames->at(i).next_packets_ptr);

      PointCloudXYZRRIAR::Ptr output_cloud;
      output_cloud.reset(new PointCloudXYZRRIAR());
      interface->ProcessScan(three_packets, output_cloud);
      
      three_packets.clear();

      pcl::PointCloud<perception_lib::PointXYZRRIARH> cloud_with_height;
      perception_lib::PointHeightProcess(*output_cloud, cloud_with_height);

  }

}


} //namespace mapping

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}