// Mapping 3.0
// Copyright (C) idriverplus(BeiJing ZhiXingZhe, Inc.)
// All rights reserved

/***********************************************
 *
 * History
 * Wang Qi          2019.02.12     original version
 * Elbert Huang     2019.08.12     version 1.1
 *
 * ********************************************/

#ifndef MAPPING_INTERFACE_H
#define MAPPING_INTERFACE_H

#include <glog/logging.h>
#include <gtest/gtest.h>

#include "common/mapping_point_types.h"
#include "common/message_def.h"
#include "common/timed_pose.h"
#include "io/yaml_io.h"
#include "point_height.h"

namespace driver::velodyne {
class VelodyneBase;
class LidarTransform;
struct HDLDataPacket;
struct HDLFiringData;
struct HDL64RawBlock;
struct HDL64RawPacket;
union RawDistance;
}  // namespace driver::velodyne

namespace mapping::tools {

class PCLPointsCompensator;

struct VelodyneConfig {
    void LoadFromYAML(const io::YAML_IO& yaml);
    int type = 16;
    bool is_irad = false;
    double max_range = 60.0;
    double min_range = 1.0;
    double max_angle;
    double min_angle;
    bool is_organized = false;
    double view_direction = 0;
    double view_width = 0;
    bool enable_coordinate_transformation = true;
    double xoffset;
    double yoffset;
    double zoffset;
    double roll;
    double pitch;
    double yaw;
    int pre_rot_axis_0 = 0;
    int pre_rot_axis_1 = 1;
    int pre_rot_axis_2 = 2;
    double pre_rot_degree_0 = 0.0;
    double pre_rot_degree_1 = 0.0;
    double pre_rot_degree_2 = 0.0;
    double car_left = 1.0;
    double car_right = -1.0;
    double car_front = 2.0;
    double car_back = -2.0;
    double car_top = 2.0;
    double car_bottom = -20.0;
};
class PerceptionInterface {
   public:
    using PointXYZRRIART = perception_lib::PointXYZRRIART;
    using PointCloudXYZRRIART = pcl::PointCloud<PointXYZRRIART>;
    using PointXYZRRIARTH = perception_lib::PointXYZRRIARTH;
    using PointCloudXYZRRIARTH = pcl::PointCloud<PointXYZRRIARTH>;

    using PointXYZRRIAR = driver::velodyne::PointXYZRRIAR;
    using PointCloudXYZRRIAR = pcl::PointCloud<driver::velodyne::PointXYZRRIAR>;

    PerceptionInterface(const VelodyneConfig& velodyne_config);
    PerceptionInterface(const io::YAML_IO& yaml);
    PerceptionInterface() {}
    ~PerceptionInterface() {}

    void ProcessScan(std::vector<PacketsMsgPtr>& packets_msg, PointCloudXYZRRIART::Ptr& output_cloud);

    void ProcessScan32(PacketsMsgPtr& packets_ptr, common::PointCloudXYZIHIRBS::Ptr& out_cloud);

    void ProcessScan64(PacketsMsgPtr& packets_ptr, common::PointCloudXYZIHIRBS::Ptr& out_cloud);

    void PointConvertAndHeightProcess(std::vector<PacketsMsgPtr>& three_packets,
                                      common::PointCloudXYZIHIRBS::Ptr& full_cloud);

    VelodyneConfig const& GetVelodyneConfig() { return velodyne_config_; }

    void SetDrPose(std::vector<common::TimedPose>& DR_pose_buffer);

    void AdjustPointClouds(PointCloudXYZRRIAR& input_cloud, common::PointCloudXYZIHIRBS::Ptr& output_cloud);

   private:
    bool ProcessPacket(const driver::velodyne::HDLDataPacket* data_packet, uint64_t nsec);

    void ProcessFiring(std::vector<PointXYZRRIART>& firing_points, const driver::velodyne::HDLFiringData* firing_data,
                       int block, int azimuth_diff, double pkt_time);

    bool ProcessPacket32(const driver::velodyne::HDLDataPacket* data_packet, uint64_t nsec);

    void ProcessFiring32(std::vector<PointXYZRRIAR>& firing_points, const driver::velodyne::HDLFiringData* firing_data,
                         int block, float azimuth_diff, double pkt_time);

    bool ProcessPacket64(const driver::velodyne::HDL64RawPacket* data_packet, const double& pkt_time);

    void ComputeCoords(const union driver::velodyne::RawDistance &raw_distance, const int &laser_number,
                       const uint16_t &rotation, PointXYZRRIAR &point,
                       const float distance_resolution = 0.002f);

    inline bool AtEndScanningBoundary(uint16_t current_azimuth) {
        if (current_azimuth >= start_angle_ && last_azimuth_ < start_angle_) return true;
        else
            return false;
    }

    inline bool AtEndScanningBoundary32(uint16_t current_azimuth) {
        if (current_azimuth >= start_angle_32_ && last_azimuth_32_ < start_angle_32_) {
            return true;
        } else {
            return false;
        }
    }

    inline bool isPointValid(const PointXYZRRIAR& point) {
        if (point.x == std::numeric_limits<float>::max() || point.x == std::numeric_limits<float>::min() ||
            point.y == std::numeric_limits<float>::max() || point.y == std::numeric_limits<float>::min() ||
            point.z == std::numeric_limits<float>::max() || point.z == std::numeric_limits<float>::min()) {
            return false;
        }

        return true;
    }

    inline bool isPointInRange(const PointXYZRRIAR& point) {
        double radius_from_lidar = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
        if (radius_from_lidar < 60 && radius_from_lidar > 1.0) {
            return true;
        }

        return false;
    }

    inline bool isPointInRange(int rotation, float range) {
        if (range < 60 || range > 1.0) {
            return true;
        }
        return false;
    }

   private:
    VelodyneConfig velodyne_config_;

    PointCloudXYZRRIART::Ptr cloud_;

    int pcd_count_ = 0;

    std::shared_ptr<tools::PCLPointsCompensator> compensator_;
    const double (*inner_time_)[12][32];

    std::shared_ptr<driver::velodyne::VelodyneBase> lidar_mode_;
    std::shared_ptr<driver::velodyne::LidarTransform> lidar_transform_;

    uint16_t last_azimuth_;
    uint16_t start_angle_;

    int block_index_ = 0;

    // 32 velodyne
    PointCloudXYZRRIAR::Ptr cloud_32_;
    std::shared_ptr<driver::velodyne::VelodyneBase> lidar_type_32_;
    std::shared_ptr<driver::velodyne::LidarTransform> lidar_transform_32_;
    const double (*inner_time_32_)[12][32];
    uint16_t last_azimuth_32_;
    uint16_t start_angle_32_;

    // 64 velodyne
    PointCloudXYZRRIAR::Ptr cloud_64_;
    std::shared_ptr<driver::velodyne::VelodyneBase> lidar_type_64_;
    std::shared_ptr<driver::velodyne::LidarTransform> lidar_transform_64_;
    const double (*inner_time_64_)[12][32];
    bool is_64e_s2_;
    bool need_two_pt_correction_ = true;

    double min_point_time_ = -1.0;
    double max_point_time_ = -1.0;
    bool enable_basetime_ = false;
};

}  // namespace mapping::tools

#endif  // MAPPING_INTERFACE_H