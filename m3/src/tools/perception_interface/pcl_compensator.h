//
// Created by elbert huang on 19-8-22.
//

#ifndef MAPPING_PCL_COMPENSATOR_H
#define MAPPING_PCL_COMPENSATOR_H

#include "common/circular_buffer.h"
#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "common/timed_pose.h"
#include "point_height.h"

namespace mapping::tools {

/// 对PCL点云进行运动补偿
class PCLPointsCompensator {
   public:
    PCLPointsCompensator() = default;
    ~PCLPointsCompensator() = default;

    void SetDrPose(const std::vector<common::TimedPose>& DR_pose_buffer);

    void SetParam(double max_range, double min_range) {
        max_range_ = max_range;
        min_range_ = min_range;
    }

    void Perform(PointCloudXYZRRIARTH::Ptr origin_cloud, common::PointCloudXYZIHIRBS::Ptr compensated_cloud);

    void ConvertPointCloud(PointCloudXYZRRIARTH::Ptr in_cloud, common::PointCloudXYZIHIRBS::Ptr out_cloud);

    void Perform32Compensation(PointCloudXYZRRIAR::Ptr origin_cloud,
                          PointCloudXYZRRIAR::Ptr compensated_cloud, const double& min_timestamp, const double& max_timestamp);

   private:
    void ComputeTimestampInterval(PointCloudXYZRRIARTH::Ptr origin_cloud, double& min_timestamp, double& max_timestamp);

    bool FindCorrespondPoseFrom(common::CircularBuffer<common::TimedPose>& pose_buffer, double min_timestamp,
                                double max_timestamp, SE3& min_time_pose, SE3& max_time_pose);

    void MotionCompensation(PointCloudXYZRRIARTH::Ptr& msg, common::PointCloudXYZIHIRBS::Ptr out_points,
                            const double min_timestamp, const double max_timestamp, const SE3& min_time_pose,
                            const SE3& max_time_pose);
    
    void MotionCompensation32(PointCloudXYZRRIAR::Ptr& msg,
                              PointCloudXYZRRIAR::Ptr& out_points,
                              const double min_timestamp,
                              const double max_timestamp,
                              const SE3& min_time_pose,
                              const SE3& max_time_pose);

    inline bool HasDrPose() { return pose_buffer_.size() > 0; }

    template<typename T>
    inline bool isPointValid(const T& point) {
        if (point.x == common::HIT_NAN || point.x == common::HIT_FREE || point.y == common::HIT_NAN ||
            point.y == common::HIT_FREE || point.z == common::HIT_NAN || point.z == common::HIT_FREE) {
            return false;
        }
        return true;
    }
 
    template<typename T>
    inline bool isPointInRange(const T& point) {
        double radius_from_lidar = sqrt(pow(point.x, 2.0) + pow(point.y, 2.0));
        if (radius_from_lidar < max_range_ && radius_from_lidar > min_range_) return true;
        return false;
    }

   private:
    common::CircularBuffer<common::TimedPose> pose_buffer_;

    int count_test = 0;
    double max_range_, min_range_;
    bool is_motion_compensation_valid_ = false;
};

}  // namespace mapping::tools
#endif  // MAPPING_PCL_COMPENSATOR_H
