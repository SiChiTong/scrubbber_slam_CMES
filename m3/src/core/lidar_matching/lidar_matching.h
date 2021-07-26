//
// Created by wangqi on 19-7-22.
//

#ifndef MAPPING_LIDAR_MATCHING_H
#define MAPPING_LIDAR_MATCHING_H

#include "common/mapping_point_types.h"
#include "common/num_type.h"

#include <map>

namespace mapping::core {

/// Lidar matching的公共接口，现有实现有NDT, Loam
class LidarMatching {
   public:
    LidarMatching(){};

    virtual ~LidarMatching() {}

   public:
    /// 设置输入点云
    virtual void SetInputCloud(const common::PointCloudType::Ptr& input, const int kf_id) {}

    virtual void SetTrajectoryID(int id) {}

    virtual void SetPreviousFrameID(int id) {}

    virtual void SetTrajChanged(bool is_changed) {}

    virtual void SetFirstFixedPose(const SE3& first_pose) {}

    virtual void SetPredictPose(const SE3& predict_pose) {}

    virtual void SetStartID(int id) {}

    virtual void Initialization() {}

    virtual void Run() {}

    virtual SE3 GetLatestKeyFramePose() { return {}; }

    virtual double GetFitnessScore() { return 0; }

    virtual std::map<int, float> GetDegeneracyEigenValue() { return {}; }
    virtual std::map<int, SE3> GetMatchingPoses() { return {}; }
    virtual std::map<int, V6d> GetMatchingNoise() { return {}; }
};

}  // namespace mapping::core

#endif  // MAPPING_LIDAR_MATCHING_H