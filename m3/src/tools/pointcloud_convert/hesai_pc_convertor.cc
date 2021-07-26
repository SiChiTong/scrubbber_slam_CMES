//
// Created by pengguoqi on 20-08-24.
//

#include "tools/pointcloud_convert/hesai_pc_convertor.h"

#include <glog/logging.h>
#include <pcl/filters/filter.h>
#include "tools/pointcloud_convert/compensator.h"

using namespace mapping::common;
using namespace hesai::lidar;

namespace mapping::tools {

/*
40P <------> lidar_type = "Pandar40P"
64  <------> lidar_type = "Pandar64"    (默认)
20A <------> lidar_type = "Pandar20A"
20B <------> lidar_type = "Pandar20B"
QT <------> lidar_type = "PandarQT"
40M <------> lidar_type = "Pandar40M"
32 <------> lidar_type = "PandarXT-32"
16 <------> lidar_type = "PandarXT-16"
*/

HeSaiPcConvertor::HeSaiPcConvertor(const HesaiConf &config) : compensator_(new Compensator) {
    std::vector<double> calib_config;
    calib_config.resize(12);
    calib_config[0] = config.car_left;
    calib_config[1] = config.car_right;
    calib_config[2] = config.car_front;
    calib_config[3] = config.car_back;
    calib_config[4] = config.car_top;
    calib_config[5] = config.car_bottom;
    calib_config[6] = config.xoffset;
    calib_config[7] = config.yoffset;
    calib_config[8] = config.zoffset;
    calib_config[9] = config.roll;
    calib_config[10] = config.pitch;
    calib_config[11] = config.yaw;
    HSINFO << " lidar_type : " << config.lidar_type << HSREND;
    hsdk_ = std::make_shared<PandarGeneralSDK>(calib_config, config.lidar_type, config.start_angle, config.tz,
                                               config.packets_size);
}

HeSaiPcConvertor::~HeSaiPcConvertor() { hsdk_ = nullptr; }

int HeSaiPcConvertor::Convert(const HeSaiScanMsg &scan, PointCloudType::Ptr &output) {
    PPointCloudPtr temp_cloud(new PPointCloud());
    output->clear();
    int re = hsdk_->Convert(scan, temp_cloud);
    if (re < 0) return re;
    ConvertCloudType(temp_cloud, output);
    return 0;
}

void HeSaiPcConvertor::ConvertCloudType(const PPointCloud::Ptr &input_cloud, PointCloudType::Ptr &output_cloud) {
    if (input_cloud->points.size() <= 0) {
        LOG(INFO) << "input cloud is empty.";
        return;
    }
    PointCloudType::Ptr temp_cloud(new PointCloudType());
    temp_cloud->clear();

    for (const auto &point : input_cloud->points) {
        if (std::isnan(point.x) || std::isnan(point.y) || isnan(point.z) || std::isinf(point.x) ||
            std::isinf(point.y) || std::isinf(point.y)) {
            continue;
        }

        PointType new_point;
        new_point.x = point.x;
        new_point.y = point.y;
        new_point.z = point.z;
        new_point.intensity = point.intensity;
        new_point.intensity_vi = point.intensity;
        temp_cloud->points.push_back(new_point);
    }

    temp_cloud->header.stamp = input_cloud->header.stamp;
    temp_cloud->height = 1;
    temp_cloud->width = temp_cloud->points.size();
    temp_cloud->is_dense = false;

    *output_cloud = *temp_cloud;
}

void HeSaiPcConvertor::SetDrPose(std::vector<common::TimedPose> &dr_pose_buffer) {
    compensator_->SetDrPose(dr_pose_buffer);
}

}  // namespace mapping::tools