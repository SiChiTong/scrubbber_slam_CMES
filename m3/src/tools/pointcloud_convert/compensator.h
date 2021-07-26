//
// Created by wangqi on 19-7-25.
//

#ifndef MAPPING_COMPENSATOR_H
#define MAPPING_COMPENSATOR_H

#include <sensor_msgs/PointCloud2.h>

#include "common/circular_buffer.h"
#include "common/message_def.h"
#include "common/num_type.h"
#include "common/timed_pose.h"

namespace mapping::tools {

class Compensator {
   public:
    Compensator();
    ~Compensator(){};

    void SetDrPose(std::vector<common::TimedPose>& DR_pose_buffer);
    void Perform(sensor_msgs::PointCloud2ConstPtr origin_cloud, sensor_msgs::PointCloud2::Ptr compensated_cloud);

    void PerformSuTeng80Compensation(RSCloudType::Ptr origin_cloud, RSCloudType::Ptr& compensated_cloud,
                                     const double& min_timestamp, const double& max_timestamp, double scan_time);
    void PerformSuTeng80Compensation(IRADCloudType::Ptr origin_cloud, IRADCloudType::Ptr& compensated_cloud,
                                     const double& min_timestamp, const double& max_timestamp, double scan_time);
   private:
    bool CheckMessage(sensor_msgs::PointCloud2ConstPtr msg);

    void ComputeTimestampInterval(sensor_msgs::PointCloud2ConstPtr origin_cloud, double& min_timestamp,
                                  double& max_timestamp);

    bool FindCorrespondPoseFrom(common::CircularBuffer<common::TimedPose>& pose_buffer, double min_timestamp,
                                double max_timestamp, Eigen::Affine3f& min_time_pose, Eigen::Affine3f& max_time_pose);

    uint GetFieldSize(const int data_type);

    template <typename Scalar>
    void MotionCompensation(sensor_msgs::PointCloud2::Ptr& msg, const double min_timestamp, const double max_timestamp,
                            const Aff3f& min_time_pose, const Aff3f& max_time_pose);

    inline bool HasDrPose() { return pose_buffer_.size() > 0; }

    void MotionCompensationSuTeng80(RSCloudType::Ptr& msg, RSCloudType::Ptr& out_points, const double min_timestamp,
                                    const double max_timestamp, const Aff3f& min_time_pose, const Aff3f& max_time_pose);
    void MotionCompensationSuTeng80(IRADCloudType::Ptr& msg, IRADCloudType::Ptr& out_points, const double min_timestamp,
                                    const double max_timestamp, const Aff3f& min_time_pose, const Aff3f& max_time_pose);
    template <typename T>
    inline bool isPointValidSuTeng(const T& point) {
        if (point.x == std::numeric_limits<float>::max() || point.x == std::numeric_limits<float>::min() ||
            point.y == std::numeric_limits<float>::max() || point.y == std::numeric_limits<float>::min() ||
            point.z == std::numeric_limits<float>::max() || point.z == std::numeric_limits<float>::min())
            return false;
        return true;
    }

    template <typename T>
    inline bool isPointInRangeSuTeng(const T& point) {
        double radius_from_lidar = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
        return (radius_from_lidar < 60 && radius_from_lidar > 1.0);
    }

   private:
    int x_offset_;
    int y_offset_;
    int z_offset_;
    int timestamp_offset_;
    uint timestamp_data_size_;

    common::CircularBuffer<common::TimedPose> pose_buffer_;

    bool is_motion_compensation_valid_;
};

}  // namespace mapping::tools
#endif  // MAPPING_COMPENSATOR_H
