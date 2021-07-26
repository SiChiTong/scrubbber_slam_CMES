//
// Created by wangqi on 19-7-19.
//

#ifndef MAPPING_RANGE_IMAGE_PROJECTION_H
#define MAPPING_RANGE_IMAGE_PROJECTION_H

#include <opencv2/core/core.hpp>
#include <vector>

#include "cloud_msgs/cloud_info.h"
#include "common/mapping_point_types.h"
#include "core/lidar_matching/feature_matching/matching_param.h"

namespace mapping::core {

class ImageProjection {
   public:
    ImageProjection(FeatureMatchingParams &params);

    ~ImageProjection();

    void SetInputCloud(const common::PointCloudType::Ptr &input_cloud_in) { *input_laser_cloud_ = *input_cloud_in; }

    void CloudHandler();

    void GetInputLaserCloud(common::PointCloudType::Ptr &input_cloud_out) { input_cloud_out = input_laser_cloud_; }

    void GetSegmentedCloud(common::PointCloudType::Ptr &segmented_cloud) { segmented_cloud = segmented_cloud_; }

    void GetOutlierCloud(common::PointCloudType::Ptr &outlier_cloud) { outlier_cloud = outlier_cloud_; }

    void GetSegInfo(cloud_msgs::cloud_info &seg_info) { seg_info = seg_msg_; }

    void ResetParameters();

   private:
    void FindStartEndAngle();

    void ProjectPointCloud();

    void GroundRemoval();

    void CloudSegmentation();

    void LabelComponents(int row, int col);

    void SetCloudTimeStamp();

   private:
    FeatureMatchingParams params_;

    common::PointCloudType::Ptr input_laser_cloud_;

    common::PointCloudType::Ptr segmented_cloud_;
    common::PointCloudType::Ptr outlier_cloud_;
    cloud_msgs::cloud_info seg_msg_;

    common::PointCloudType::Ptr full_cloud_;
    common::PointCloudType::Ptr full_info_cloud_;

    common::PointType nan_point_;

    std::vector<std::pair<uint8_t, uint8_t>> neighbor_iterator_;

    std::vector<uint16_t> all_pushed_ind_x_;
    std::vector<uint16_t> all_pushed_ind_y_;
    std::vector<uint16_t> queue_ind_x_;
    std::vector<uint16_t> queue_ind_y_;

    cv::Mat range_mat_;
    cv::Mat label_mat_;
    cv::Mat ground_mat_;
    int label_count_;

    double cloud_z_offset_;
};

}  // namespace mapping::core

#endif  // MAPPING_RANGE_IMAGE_PROJECTION_H