//
// Created by pengguoqi on 21-01-28.
//

#include "tools/pointcloud_convert/suteng_irad_pc_convertor.h"
#include "common/mapping_point_types.h"
#include "tools/pointcloud_convert/compensator.h"

#include <glog/logging.h>

namespace mapping::tools {

using namespace avos::driver;

SuTengIRADPcConvertor::SuTengIRADPcConvertor()
    : compensator_(new Compensator) {
    driver_version_ = "v1.1.0";
    LOG(INFO) << " driver version : " << driver_version_;
}

SuTengIRADPcConvertor::~SuTengIRADPcConvertor() { manager_ptr_ = nullptr; }

int SuTengIRADPcConvertor::Init(const common::VehicleCalibrationParam &config) {
    manager_ptr_ = std::make_shared<RsLidarApi>();
    DecoderConfig decoder_config;
    decoder_config.tf_x = config.perception_lidar_x_offset_top_center;
    decoder_config.tf_y = config.perception_lidar_y_offset_top_center;
    decoder_config.tf_z = config.perception_lidar_z_offset_top_center;
    decoder_config.tf_roll = config.perception_lidar_roll_top_center;
    decoder_config.tf_pitch = config.perception_lidar_pitch_top_center;
    decoder_config.tf_yaw = config.perception_lidar_yaw_top_center;
    decoder_config.pre_rot_axis_0 = config.pre_rot_axis_0;
    decoder_config.pre_rot_axis_1 = config.pre_rot_axis_1;
    decoder_config.pre_rot_axis_2 = config.pre_rot_axis_2;
    decoder_config.pre_rot_degree_0 = config.pre_rot_degree_0;
    decoder_config.pre_rot_degree_1 = config.pre_rot_degree_1;
    decoder_config.pre_rot_degree_2 = config.pre_rot_degree_2;
    manager_ptr_->Init(decoder_config);
    return 0;
}

std::string SuTengIRADPcConvertor::GetSutengLidarType(){
    std::string suteng_type = "RS80";
    return suteng_type;
}

int SuTengIRADPcConvertor::Convert(const SuTengIRADPacketsMsg &packet,
                                   common::PointCloudType::Ptr &output) {
    LOG(INFO) << std::setprecision(16) <<"packet header: "<<packet.header.stamp.toSec();
    IRADCloudType::Ptr temp_cloud(new IRADCloudType);
    output->clear();
    int re = manager_ptr_->Convert(packet, temp_cloud);
    if (re < 0) {
        return re;
    }

    IRADCloudType::Ptr temp_cloud_motion(new IRADCloudType);
    MotionCompensationWithTimeStamp(temp_cloud, temp_cloud_motion, packet.header.stamp.toSec());

    ConvertCloudType(temp_cloud_motion, output);
    return 0;
}

void SuTengIRADPcConvertor::SetDrPose(std::vector<common::TimedPose> &dr_pose_buffer) {
    compensator_->SetDrPose(dr_pose_buffer);
}

void SuTengIRADPcConvertor::ConvertCloudType(IRADCloudType::Ptr input_cloud, common::PointCloudType::Ptr &output_cloud) {
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

void SuTengIRADPcConvertor::MotionCompensationWithTimeStamp(const IRADCloudType::Ptr &input_cloud,
                                                            IRADCloudType::Ptr &temp_cloud_motion, double packet_header_time) {
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

    compensator_->PerformSuTeng80Compensation(input_cloud, temp_cloud_motion, min_timestamp, max_timestamp, packet_header_time);

    //LOG(INFO) << "SuTeng Timestamp: "<<std::setprecision(16)<<first_timestamp<<" "<<last_timestamp<<" "<<min_timestamp<<" "<<max_timestamp;
}

}  // namespace mapping::tools