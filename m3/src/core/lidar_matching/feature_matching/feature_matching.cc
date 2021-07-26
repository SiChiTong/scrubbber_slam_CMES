//
// Created by wangqi on 19-7-19.
//
#include <memory>

#include "core/lidar_matching/feature_matching/feature_extractor.h"
#include "core/lidar_matching/feature_matching/feature_matching.h"
#include "core/lidar_matching/feature_matching/feature_tracker.h"
#include "core/lidar_matching/feature_matching/local_mapper.h"

namespace mapping::core {

FeatureMatching::FeatureMatching(FeatureMatchingParams& params)
    : params_(params),
      feature_extractor_(new FeatureExtractor(params)),
      tracker_(new FeatureTracker(params)),
      local_mapper_(new LocalMapper(params)) {}

FeatureMatching::~FeatureMatching() {}

void FeatureMatching::Initialization() {
    feature_extractor_ = std::make_shared<FeatureExtractor>(params_);
    tracker_ = std::make_shared<FeatureTracker>(params_);
    local_mapper_ = std::make_shared<LocalMapper>(params_);
    local_mapper_->SetTrajectoryID(trajectory_id_);

    keyframe_id_ = -1;
}

void FeatureMatching::SetTrajectoryID(int id) {
    trajectory_id_ = id;
    local_mapper_->SetTrajectoryID(trajectory_id_);
}

void FeatureMatching::SetInputCloud(const common::PointCloudType::Ptr& input, const int kf_id) {
    feature_extractor_->SetInputCloud(input);
    keyframe_id_ = kf_id;
}

void FeatureMatching::SetPreviousFrameID(int id) { local_mapper_->SetPreviousFrameID(id); }

void FeatureMatching::SetFirstFixedPose(const SE3& first_pose) { local_mapper_->SetFirstFixedPose(first_pose); }

void FeatureMatching::SetPredictPose(const SE3& predict_pose) { tracker_->SetPredictPose(predict_pose); }

void FeatureMatching::SetStartID(int id) { local_mapper_->SetStartID(id); }

SE3 FeatureMatching::GetLatestKeyFramePose() { return local_mapper_->GetLatestKeyFramePose(); }

double FeatureMatching::GetFitnessScore() { return local_mapper_->GetFitnessScore(); }

std::map<int, float> FeatureMatching::GetDegeneracyEigenValue() { return local_mapper_->GetDegeneracyEigenValue(); }

std::map<int, SE3> FeatureMatching::GetMatchingPoses() { return local_mapper_->GetMatchingPoses(); }

std::map<int, V6d> FeatureMatching::GetMatchingNoise() { return local_mapper_->GetMatchingNoise(); }

void FeatureMatching::ExtractFeatures() { feature_extractor_->Run(); }

void FeatureMatching::TrackFeatures() {
    tracker_->SetCornerPointsSharp(feature_extractor_->GetCornerPointsSharp());
    tracker_->SetCornerPointsLessSharp(feature_extractor_->GetCornerPointsLessSharp());
    tracker_->SetSurfPointsFlat(feature_extractor_->GetSurfPointsFlat());
    tracker_->SetSurfPointsLessFlat(feature_extractor_->GetSurfPointsLessFlat());
    tracker_->SetOutlierCloud(feature_extractor_->GetOutlierCloud());

    if (!tracker_->system_inited_LM_) {
        tracker_->CheckSystemInitialization();
        return;
    }

    tracker_->Tracking();
}

void FeatureMatching::LocalMapping() {
    local_mapper_->SetCornerPointLast(tracker_->GetCornerPointLast());
    local_mapper_->SetSurfPointLast(tracker_->GetSurfPointLast());
    local_mapper_->SetOutlierCloud(tracker_->GetOutlierCloud());
    local_mapper_->SetCurrentTransform(tracker_->GetCurrentTransform());
    local_mapper_->SetKeyFrameID(keyframe_id_);

    local_mapper_->Run();
}

void FeatureMatching::Run() {
    ExtractFeatures();
    TrackFeatures();
    LocalMapping();
}

}  // namespace mapping::core