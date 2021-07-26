//
// Created by wangqi on 19-7-19.
//

#ifndef MAPPING_FEATURE_MATCHING_H
#define MAPPING_FEATURE_MATCHING_H

#include "core/lidar_matching/feature_matching/matching_param.h"
#include "core/lidar_matching/lidar_matching.h"

namespace mapping::core {

class FeatureTracker;
class FeatureExtractor;
class LocalMapper;

/// 利用特征法进行点云匹配，from lego-loam
class FeatureMatching : public LidarMatching {
   public:
    FeatureMatching(FeatureMatchingParams& params);

    virtual ~FeatureMatching();

    virtual void SetTrajectoryID(int id);

    virtual void SetInputCloud(const common::PointCloudType::Ptr& input, const int kf_id);

    virtual void SetPreviousFrameID(int id) override;

    virtual void SetFirstFixedPose(const SE3& first_pose) override;

    virtual void SetPredictPose(const SE3& predict_pose) override;

    virtual void SetStartID(int id) override;

    virtual void Run() override;

    virtual SE3 GetLatestKeyFramePose() override;

    virtual double GetFitnessScore() override;

    virtual std::map<int, float> GetDegeneracyEigenValue() override;

    virtual std::map<int, SE3> GetMatchingPoses() override;

    virtual std::map<int, V6d> GetMatchingNoise() override;

   private:
    void Initialization();

    void ExtractFeatures();

    void TrackFeatures();

    void LocalMapping();

   private:
    FeatureMatchingParams params_;

    int trajectory_id_ = -1;
    int keyframe_id_ = -1;

    std::shared_ptr<FeatureExtractor> feature_extractor_ = nullptr;
    std::shared_ptr<FeatureTracker> tracker_ = nullptr;
    std::shared_ptr<LocalMapper> local_mapper_ = nullptr;
};

}  // namespace mapping::core

#endif  // MAPPING_FEATURE_MATCHING_H
