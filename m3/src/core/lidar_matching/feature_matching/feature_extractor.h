//
// Created by wangqi on 19-7-19.
//

#ifndef MAPPING_FEATURE_EXTRACTOR
#define MAPPING_FEATURE_EXTRACTOR

#include <pcl/filters/voxel_grid.h>
#include "common/mapping_point_types.h"
#include "core/lidar_matching/feature_matching/matching_param.h"
#include "core/lidar_matching/feature_matching/range_image_projection.h"

namespace mapping::core {

class FeatureExtractor {
   public:
    // Constructor
    FeatureExtractor(FeatureMatchingParams &params);

    ~FeatureExtractor();

    void SetInputCloud(const common::PointCloudType::Ptr &input_cloud) {
        image_projection_->SetInputCloud(input_cloud);
    }

    void Initialization();

    void Run();

    common::PointCloudType::Ptr GetCornerPointsSharp() { return corner_points_sharp_; }

    common::PointCloudType::Ptr GetCornerPointsLessSharp() { return corner_points_less_sharp_; }

    common::PointCloudType::Ptr GetSurfPointsFlat() { return surf_points_flat_; }

    common::PointCloudType::Ptr GetSurfPointsLessFlat() { return surf_points_less_flat_; }

    common::PointCloudType::Ptr GetOutlierCloud() { return outlier_cloud_; }

    std::shared_ptr<ImageProjection> GetImageProjection() { return image_projection_; }

    struct Smoothness {
        float value;
        size_t ind;
    };

    struct ByValue {
        bool operator()(Smoothness const &left, Smoothness const &right) {
            return left.value < right.value;
        }
    };

   private:
    void PointSegmentation();

    void AdjustDistortion();

    void CalculateSmoothness();

    void MarkOccludedPoints();

    void ExtractFeatures();

    void SetCloudTimeStamp();

   private:
    FeatureMatchingParams params_;

    common::PointCloudType::Ptr segmented_cloud_;
    common::PointCloudType::Ptr outlier_cloud_;
    cloud_msgs::cloud_info seg_info_;

    common::PointCloudType::Ptr corner_points_sharp_;
    common::PointCloudType::Ptr corner_points_less_sharp_;
    common::PointCloudType::Ptr surf_points_flat_;
    common::PointCloudType::Ptr surf_points_less_flat_;

    common::PointCloudType::Ptr surf_points_less_flat_scan_;
    common::PointCloudType::Ptr surf_points_less_flat_scan_DS_;

    std::vector<Smoothness> cloud_smoothness_;

    std::vector<float> cloud_curvature_;
    std::vector<int> cloud_neighbor_picked_;
    std::vector<int> cloud_label_;

    pcl::VoxelGrid<common::PointType> downsize_filter_;

    std::shared_ptr<ImageProjection> image_projection_;
};
}  // namespace mapping::core

#endif  // MAPPING_FEATURE_EXTRACTOR