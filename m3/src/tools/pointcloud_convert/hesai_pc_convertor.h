
//
// Created by pengguoqi on 20-08-24.
//

#ifndef HESAI_PC_CONVERTOR_H
#define HESAI_PC_CONVERTOR_H

#include "common/message_def.h"
#include "common/timed_pose.h"

namespace mapping::tools {

class Compensator;

struct HesaiConf {
    std::string lidar_type;
    double car_left;
    double car_right;
    double car_front;
    double car_back;
    double car_top;
    double car_bottom;
    double xoffset;
    double yoffset;
    double zoffset;
    double roll;
    double pitch;
    double yaw;
    uint16_t start_angle;
    int tz;
    int packets_size;

    HesaiConf() {
        lidar_type = "Pandar64";
        xoffset = 0.0;
        yoffset = 0.0;
        zoffset = 0.0;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        car_left = 0.0;
        car_right = 0.0;
        car_front = 0.0;
        car_back = 0.0;
        car_top = 0.0;
        car_bottom = 0.0;
        start_angle = 0;
        tz = 0;
        packets_size = 300;
    }
};

//禾赛输出的packets转换成pointcloud格式
class HeSaiPcConvertor {
   public:
    explicit HeSaiPcConvertor(const HesaiConf &config);
    virtual ~HeSaiPcConvertor();

    int Convert(const HeSaiScanMsg &scan, common::PointCloudType::Ptr &output);

    void SetDrPose(std::vector<common::TimedPose> &dr_pose_buffer);

   private:
    void ConvertCloudType(const hesai::lidar::PPointCloud::Ptr &input_cloud, common::PointCloudType::Ptr &output_cloud);

   private:
    std::shared_ptr<hesai::lidar::PandarGeneralSDK> hsdk_;

    std::shared_ptr<Compensator> compensator_;
};

}  // namespace mapping::tools

#endif  // HESAI_PC_CONVERTOR_H
