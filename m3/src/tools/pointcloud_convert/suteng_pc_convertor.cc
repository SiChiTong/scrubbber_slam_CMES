//
// Created by pengguoqi on 20-08-12.
//

#include "tools/pointcloud_convert/suteng_pc_convertor.h"
#include "common/mapping_point_types.h"
#include "tools/pointcloud_convert/compensator.h"

#include <glog/logging.h>

namespace mapping::tools {

using namespace robosense::lidar;

SuTengPcConvertor::SuTengPcConvertor(const std::string &config_path)
    : config_path_(config_path), compensator_(new Compensator) {
    driver_version_ = "v1.1.0";
    LOG(INFO) << " driver version : " << driver_version_;
}

SuTengPcConvertor::~SuTengPcConvertor() { manager_ptr_ = nullptr; }

int SuTengPcConvertor::Init() {
    YAML::Node config;
    LOG(INFO) << " config_path : " << config_path_;
    try {
        config = YAML::LoadFile(config_path_);
    } catch (...) {
        LOG(ERROR) << " Config file format wrong! Please check the format or intendation! ";
        return -1;
    }
    manager_ptr_ = std::make_shared<Manager>();
    if (manager_ptr_->Init(config) < 0) {
        LOG(ERROR) << " Manager init failed! ";
        return -2;
    }
    return 0;
}

std::string SuTengPcConvertor::GetSutengLidarType(){
    std::string suteng_type = "RS80";
    if(manager_ptr_){
        suteng_type = manager_ptr_->GetSutengLidarType();
    } else {
        LOG(ERROR) << "Manager not initial!";
    }
    return suteng_type;
}

int SuTengPcConvertor::Convert(const SuTengScanMsg &scan, const SuTengPacketsMsg &packet,
                               common::PointCloudType::Ptr &output) {
    LOG(INFO) << std::setprecision(16) <<"scan msg header: "<<scan.header.stamp.toSec();
    RSCloudType::Ptr temp_cloud(new RSCloudType);
    output->clear();
    int re = manager_ptr_->Convert(scan, packet, temp_cloud);
    if (re < 0) {
        return re;
    }

    RSCloudType::Ptr temp_cloud_motion(new RSCloudType);
    MotionCompensationWithTimeStamp(temp_cloud, temp_cloud_motion, scan.header.stamp.toSec());

    ConvertCloudType(temp_cloud_motion, output);
    return 0;
}

void SuTengPcConvertor::SetDrPose(std::vector<common::TimedPose> &dr_pose_buffer) {
    compensator_->SetDrPose(dr_pose_buffer);
}

void SuTengPcConvertor::ConvertCloudType(RSCloudType::Ptr input_cloud, common::PointCloudType::Ptr &output_cloud) {
    if (input_cloud->points.size() <= 0) {
        LOG(INFO) << "input cloud is empty.";
        return;
    }

    common::PointCloudType::Ptr temp_cloud(new common::PointCloudType());
    temp_cloud->clear();

    for (const auto &point : input_cloud->points) {
        if (std::isnan(point.x) || std::isnan(point.y) || isnan(point.z) || std::isinf(point.x) ||
            std::isinf(point.y) || std::isinf(point.y)) {
            continue;
        }

        common::PointType new_point;
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

void SuTengPcConvertor::MotionCompensationWithTimeStamp(const RSCloudType::Ptr &input_cloud,
                                                        RSCloudType::Ptr &temp_cloud_motion, double scan_header_time) {
    if (input_cloud->points.size() <= 0) {
        LOG(INFO) << "input cloud is empty.";
        return;
    }

    // LOG(INFO) << "input cloud size: "<<input_cloud->points.size();

    double first_timestamp = 0.0;
    double last_timestamp = 0.0;
    double min_timestamp = 0.0;
    double max_timestamp = 0.0;
    for (const auto &point : input_cloud->points) {
        last_timestamp = point.timestamp;
        if (first_timestamp < 1.0) {
            first_timestamp = last_timestamp;
            min_timestamp = last_timestamp + 1000.0;
        }

        if (last_timestamp < min_timestamp) {
            min_timestamp = last_timestamp;
        }

        if (last_timestamp > max_timestamp) {
            max_timestamp = last_timestamp;
        }
    }

    compensator_->PerformSuTeng80Compensation(input_cloud, temp_cloud_motion, min_timestamp, max_timestamp, scan_header_time);

    //LOG(INFO) << "SuTeng Timestamp: "<<std::setprecision(16)<<first_timestamp<<" "<<last_timestamp<<" "<<min_timestamp<<" "<<max_timestamp;
}

}  // namespace mapping::tools