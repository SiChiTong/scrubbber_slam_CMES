//
// Created by wangqi on 19-7-17.
//

#ifndef MAPPING_VELODYNE_POINTCLOUD_POINT_TYPES_H
#define MAPPING_VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>

/// PCL点云类型定义
namespace mapping::common {

struct PointXYZIT {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    float angle;
    uint8_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

} EIGEN_ALIGN16;

const float HIT_NAN = std::numeric_limits<float>::max();
const float HIT_FREE = std::numeric_limits<float>::min();

struct PointXYZIRPYT {
    PCL_ADD_POINT4D

    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

struct PointXYZIHIRBS {
    PCL_ADD_POINT4D;
    float height;
    float range;
    uint8_t intensity;
    uint8_t intensity_vi;
    uint8_t ring;
    uint8_t type;
    inline PointXYZIHIRBS() {
        x = y = z = 0.0f;
        intensity = intensity_vi = ring = type = 0;
        height = range = 0.0f;
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef pcl::PointXYZ PointXYZ;
typedef pcl::PointCloud<PointXYZ> PointCloudXYZ;
typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointCloud<PointXYZI> PointCloudXYZI;
typedef pcl::PointCloud<PointXYZI>::Ptr CloudPtr;
typedef pcl::PointCloud<PointXYZIT> PointCloudXYZIT;
typedef pcl::PointCloud<PointXYZIRPYT> PointCloudXYZIRPYT;
typedef pcl::PointCloud<PointXYZIHIRBS> PointCloudXYZIHIRBS;

/// 点云类型定义
typedef PointXYZIHIRBS PointType;
typedef pcl::PointCloud<PointType> PointCloudType;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;

}  // namespace mapping::common

POINT_CLOUD_REGISTER_POINT_STRUCT(mapping::common::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
                                      float, angle, angle)(std::uint8_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(mapping::common::PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(mapping::common::PointXYZIHIRBS,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
                                      float, height, height)(std::uint8_t, intensity_vi, intensity_vi)(
                                      float, range, range)(std::uint8_t, ring, ring)(std::uint8_t, type, type))

#endif  // MAPPING_VELODYNE_POINTCLOUD_POINT_TYPES_H
