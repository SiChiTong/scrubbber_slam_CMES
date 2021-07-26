//
// Created by wangqi on 19-7-19.
//

#ifndef MAPPING_FEATURE_TRACKER_H
#define MAPPING_FEATURE_TRACKER_H

#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/core/core.hpp>

#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "core/lidar_matching/feature_matching/matching_param.h"

namespace mapping::core {

/// TODO 我不喜欢这里的pose表达方式，把它改成统一的 @wangqi
class FeatureTracker {
   public:
    explicit FeatureTracker(FeatureMatchingParams &params);

    ~FeatureTracker();

    void SetCornerPointsSharp(const common::PointCloudType::Ptr &corner_points_sharp) {
        corner_points_sharp_ = corner_points_sharp;
    }

    void SetCornerPointsLessSharp(const common::PointCloudType::Ptr &corner_points_less_sharp) {
        corner_points_less_sharp_ = corner_points_less_sharp;
    }

    void SetSurfPointsFlat(const common::PointCloudType::Ptr &surf_points_flat) {
        surf_points_flat_ = surf_points_flat;
    }

    void SetSurfPointsLessFlat(const common::PointCloudType::Ptr &surf_points_less_flat) {
        surf_points_less_flat_ = surf_points_less_flat;
    }

    void SetOutlierCloud(const common::PointCloudType::Ptr &outlier_cloud_in) {
        outlier_cloud_ = outlier_cloud_in;
        AdjustDistortionOutlierCloud();
    }

    void SetPredictPose(const SE3 &predict_pose);

    void CheckSystemInitialization();

    void Initialization();

    void Tracking();

    common::PointCloudType::Ptr GetCornerPointLast() { return laser_cloud_corner_last_; }

    common::PointCloudType::Ptr GetSurfPointLast() { return laser_cloud_surf_last_; }

    common::PointCloudType::Ptr GetOutlierCloud() { return outlier_cloud_; }

    float *GetCurrentTransform() { return transform_sum_; }

   private:
    void TransformToStart(common::PointType const *const pi, common::PointType *const po);

    void TransformToEnd(common::PointType const *const pi, common::PointType *const po);

    void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, float &ox, float &oy,
                            float &oz);

    void FindCorrespondingCornerFeatures(int iterCount);

    void FindCorrespondingSurfFeatures(int iterCount);

    bool CalculateTransformationSurf(int iterCount);

    bool CalculateTransformationCorner(int iterCount);

    void UpdateTransformation();

    void IntegrateTransformation();

    void AdjustDistortionOutlierCloud();

    void AdjustDistortionCloudsLast();

    void SetCloudTimeStamp();

    inline double rad2deg(double radians) { return radians * 180.0 / M_PI; }

    inline double deg2rad(double degrees) { return degrees * M_PI / 180.0; }

   public:
    bool system_inited_LM_ = false;

   private:
    FeatureMatchingParams params_;

    common::PointCloudType::Ptr corner_points_sharp_;
    common::PointCloudType::Ptr corner_points_less_sharp_;
    common::PointCloudType::Ptr surf_points_flat_;
    common::PointCloudType::Ptr surf_points_less_flat_;

    common::PointCloudType::Ptr laser_cloud_corner_last_;
    common::PointCloudType::Ptr laser_cloud_surf_last_;
    common::PointCloudType::Ptr outlier_cloud_;

    common::PointCloudType::Ptr laser_cloud_ori_;
    common::PointCloudType::Ptr coeff_sel_;

    pcl::KdTreeFLANN<common::PointType>::Ptr kdtree_corner_last_;
    pcl::KdTreeFLANN<common::PointType>::Ptr kdtree_surf_last_;

    int laser_cloud_corner_last_num_;
    int laser_cloud_surf_last_num_;

    std::vector<float> point_search_corner_index1_;
    std::vector<float> point_search_corner_index2_;

    std::vector<float> point_search_surf_index1_;
    std::vector<float> point_search_surf_index2_;
    std::vector<float> point_search_surf_index3_;

    bool is_degenerate_ = false;

    float transform_current_[6];
    float transform_sum_[6];

    std::vector<int> point_search_index_;
    std::vector<float> point_search_dis_;

    common::PointType point_ori_, point_sel_, tripod1_, tripod2_, tripod3_, coeff_;

    cv::Mat matP_;
};
}  // namespace mapping::core

#endif  // MAPPING_FEATURE_TRACKER_H