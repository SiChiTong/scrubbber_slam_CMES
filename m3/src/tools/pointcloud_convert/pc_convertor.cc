//
// Created by idriver on 19-7-19.
//

#include "pc_convertor.h"

using namespace mapping::common;

namespace mapping {
namespace tools {

PcConvertor::PcConvertor(VelodyneConfig &config) : velodyne_config_(config) {
    packets_parser_ = std::make_shared<PacketsParser>();
    if (packets_parser_->Setup(velodyne_config_) < 0) {
        LOG(ERROR) << "velodyne pointcloud initialize rawdata ERROR!!!";
        return;
    }

    converted_cloud_.reset(new PointCloudXYZIT());
    compensated_cloud_.reset(new PointCloudXYZIT());

    compensator_ = std::make_shared<Compensator>();
}

void PcConvertor::SetMotionCompensatedEnableFlag(bool flag) {
    is_motion_compensation_enabled_ = flag;
}

VelodyneConfig PcConvertor::GetConfig() const { return velodyne_config_; }

void PcConvertor::SetDrPose(std::vector<TimedPose> &DR_pose_buffer) {
    compensator_->SetDrPose(DR_pose_buffer);
}

void PcConvertor::ProcessScan(const PacketsMsgPtr &packets_msg,
                              PointCloudXYZIT::Ptr &out_cloud) {
    converted_cloud_->clear();
    packets_parser_->PaddingPointCloud(packets_msg, converted_cloud_);

    if (converted_cloud_->points.size() <= 0) {
        LOG(WARNING) << "converted cloud is empty!";
        return;
    }

    if (is_motion_compensation_enabled_) {
        sensor_msgs::PointCloud2::Ptr converted_cloud_temp(
                new sensor_msgs::PointCloud2());
        sensor_msgs::PointCloud2::Ptr compensated_cloud_temp(
                new sensor_msgs::PointCloud2());
        compensated_cloud_->clear();

        pcl::toROSMsg(*converted_cloud_, *converted_cloud_temp);
        compensator_->Perform(converted_cloud_temp, compensated_cloud_temp);
        pcl::fromROSMsg(*compensated_cloud_temp, *compensated_cloud_);

        *out_cloud = *compensated_cloud_;
    } else {
        *out_cloud = *converted_cloud_;
    }
}

void PcConvertor::ProcessScan(const PacketsMsgPtr &packets_msg,
                              PointCloudType::Ptr &out_cloud) {
    PointCloudXYZIT::Ptr temp_cloud(new PointCloudXYZIT());
    out_cloud->clear();

    ProcessScan(packets_msg, temp_cloud);

    ConvertCloudType(temp_cloud, out_cloud);
}

void PcConvertor::ConvertCloudType(const PointCloudXYZIT::Ptr &input_cloud,
                                   PointCloudType::Ptr &output_cloud) {
    if (input_cloud->points.size() <= 0) {
        LOG(INFO) << "input cloud is empty.";
        return;
    }

    for (const auto &point : input_cloud->points) {
        PointType new_point;
        new_point.x = point.x;
        new_point.y = point.y;
        new_point.z = point.z;
        new_point.intensity = point.intensity;
        output_cloud->points.push_back(new_point);
    }

    output_cloud->header.stamp = input_cloud->header.stamp;
    output_cloud->height = 1;
    output_cloud->width = output_cloud->points.size();
}

}
}