#include "core/lidar_matching/feature_matching/feature_extractor.h"
#include "common/constants.h"

namespace mapping::core {

using namespace mapping::common;

FeatureExtractor::FeatureExtractor(FeatureMatchingParams &params)
    : params_(params), image_projection_(new ImageProjection(params)) {
    Initialization();
}

FeatureExtractor::~FeatureExtractor() {
    cloud_label_.clear();
    cloud_neighbor_picked_.clear();
    cloud_smoothness_.clear();
    cloud_curvature_.clear();
}

void FeatureExtractor::Initialization() {
    segmented_cloud_.reset(new PointCloudType());
    outlier_cloud_.reset(new PointCloudType());
    corner_points_sharp_.reset(new PointCloudType());
    corner_points_less_sharp_.reset(new PointCloudType());
    surf_points_flat_.reset(new PointCloudType());
    surf_points_less_flat_.reset(new PointCloudType());
    surf_points_less_flat_scan_.reset(new PointCloudType());
    surf_points_less_flat_scan_DS_.reset(new PointCloudType());

    cloud_smoothness_.resize(16 * 1800);
    cloud_curvature_.resize(16 * 1800);
    cloud_neighbor_picked_.resize(16 * 1800);
    cloud_label_.resize(16 * 1800);

    downsize_filter_.setLeafSize(0.2, 0.2, 0.2);
}

void FeatureExtractor::PointSegmentation() { image_projection_->CloudHandler(); }

void FeatureExtractor::AdjustDistortion() {
    bool half_passed = false;
    int cloud_size = segmented_cloud_->points.size();

    common::PointType point;

    for (int i = 0; i < cloud_size; i++) {
        point.x = segmented_cloud_->points[i].y;
        point.y = segmented_cloud_->points[i].z;
        point.z = segmented_cloud_->points[i].x;

        float ori = -atan2(point.x, point.z);
        if (!half_passed) {
            if (ori < seg_info_.start_orientation - kPI / 2) ori += 2 * kPI;
            else if (ori > seg_info_.start_orientation + kPI * 3 / 2)
                ori -= 2 * kPI;

            if (ori - seg_info_.start_orientation > kPI) half_passed = true;
        } else {
            ori += 2 * kPI;

            if (ori < seg_info_.end_orientation - kPI * 3 / 2) ori += 2 * kPI;
            else if (ori > seg_info_.end_orientation + kPI / 2)
                ori -= 2 * kPI;
        }

        float relative_time = (ori - seg_info_.start_orientation) / seg_info_.orientation_diff;
        point.range = int(segmented_cloud_->points[i].range) + params_.scan_period * relative_time;
        point.intensity = int(segmented_cloud_->points[i].intensity) + params_.scan_period * relative_time;
        segmented_cloud_->points[i] = point;
    }
}

void FeatureExtractor::CalculateSmoothness() {
    int cloud_size = segmented_cloud_->points.size();
    for (int i = 5; i < cloud_size - 5; i++) {
        float diff_range = seg_info_.segmented_cloud_range[i - 5] + seg_info_.segmented_cloud_range[i - 4] +
                           seg_info_.segmented_cloud_range[i - 3] + seg_info_.segmented_cloud_range[i - 2] +
                           seg_info_.segmented_cloud_range[i - 1] - seg_info_.segmented_cloud_range[i] * 10 +
                           seg_info_.segmented_cloud_range[i + 1] + seg_info_.segmented_cloud_range[i + 2] +
                           seg_info_.segmented_cloud_range[i + 3] + seg_info_.segmented_cloud_range[i + 4] +
                           seg_info_.segmented_cloud_range[i + 5];

        assert(i >= 0 && i < 16 * 1800);
        cloud_curvature_[i] = diff_range * diff_range;
        cloud_neighbor_picked_[i] = 0;
        cloud_label_[i] = 0;
        cloud_smoothness_[i].value = cloud_curvature_[i];
        cloud_smoothness_[i].ind = i;
    }
}

void FeatureExtractor::MarkOccludedPoints() {
    int cloud_size = segmented_cloud_->points.size();

    for (int i = 5; i < cloud_size - 6; ++i) {
        float depth1 = seg_info_.segmented_cloud_range[i];
        float depth2 = seg_info_.segmented_cloud_range[i + 1];
        int column_diff =
            std::abs(int(seg_info_.segmented_cloud_col_index[i + 1] - seg_info_.segmented_cloud_col_index[i]));

        if (column_diff < 10) {
            if (depth1 - depth2 > 0.3) {
                cloud_neighbor_picked_[i - 5] = 1;
                cloud_neighbor_picked_[i - 4] = 1;
                cloud_neighbor_picked_[i - 3] = 1;
                cloud_neighbor_picked_[i - 2] = 1;
                cloud_neighbor_picked_[i - 1] = 1;
                cloud_neighbor_picked_[i] = 1;
            } else if (depth2 - depth1 > 0.3) {
                cloud_neighbor_picked_[i + 1] = 1;
                cloud_neighbor_picked_[i + 2] = 1;
                cloud_neighbor_picked_[i + 3] = 1;
                cloud_neighbor_picked_[i + 4] = 1;
                cloud_neighbor_picked_[i + 5] = 1;
                cloud_neighbor_picked_[i + 6] = 1;
            }
        }

        float diff1 = std::abs(seg_info_.segmented_cloud_range[i - 1] - seg_info_.segmented_cloud_range[i]);
        float diff2 = std::abs(seg_info_.segmented_cloud_range[i + 1] - seg_info_.segmented_cloud_range[i]);

        if (diff1 > 0.02 * seg_info_.segmented_cloud_range[i] && diff2 > 0.02 * seg_info_.segmented_cloud_range[i]) {
            cloud_neighbor_picked_[i] = 1;
        }
    }
}

void FeatureExtractor::ExtractFeatures() {
    corner_points_sharp_->clear();
    corner_points_less_sharp_->clear();
    surf_points_flat_->clear();
    surf_points_less_flat_->clear();

    for (int i = 0; i < 16; i++) {
        surf_points_less_flat_scan_->clear();

        for (int j = 0; j < 6; j++) {
            int sp = (seg_info_.start_ring_index[i] * (6 - j) + seg_info_.end_ring_index[i] * j) / 6;
            int ep = (seg_info_.start_ring_index[i] * (5 - j) + seg_info_.end_ring_index[i] * (j + 1)) / 6 - 1;

            if (sp >= ep) continue;

            std::sort(cloud_smoothness_.begin() + sp, cloud_smoothness_.begin() + ep, ByValue());

            int largest_picked_num = 0;
            for (int k = ep; k >= sp; k--) {
                int ind = cloud_smoothness_[k].ind;

                bool corner_points_extraction_condition = false;
                if (params_.segment_enabled) {
                    corner_points_extraction_condition =
                        (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] > params_.edge_th &&
                         seg_info_.segmented_cloud_ground_flag[ind] == false);
                } else {
                    corner_points_extraction_condition =
                        (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] > params_.edge_th);
                }

                if (corner_points_extraction_condition) {
                    largest_picked_num++;
                    if (largest_picked_num <= 2) {
                        cloud_label_[ind] = 2;
                        corner_points_sharp_->push_back(segmented_cloud_->points[ind]);
                        corner_points_less_sharp_->push_back(segmented_cloud_->points[ind]);
                    } else if (largest_picked_num <= 20) {
                        cloud_label_[ind] = 1;
                        corner_points_less_sharp_->push_back(segmented_cloud_->points[ind]);
                    } else {
                        break;
                    }

                    cloud_neighbor_picked_[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        if (ind + l >= int(seg_info_.segmented_cloud_col_index.size())) break;
                        int column_diff = std::abs(int(seg_info_.segmented_cloud_col_index[ind + l] -
                                                       seg_info_.segmented_cloud_col_index[ind + l - 1]));
                        if (column_diff > 10) break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        if (ind + l < 0) break;
                        int column_diff = std::abs(int(seg_info_.segmented_cloud_col_index[ind + l] -
                                                       seg_info_.segmented_cloud_col_index[ind + l + 1]));
                        if (column_diff > 10) break;
                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                }
            }

            int smallest_picked_num = 0;
            for (int k = sp; k <= ep; k++) {
                int ind = cloud_smoothness_[k].ind;

                bool surf_points_extraction_condition = false;
                if (params_.segment_enabled) {
                    surf_points_extraction_condition =
                        (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] < params_.surf_th &&
                         seg_info_.segmented_cloud_ground_flag[ind] == true);
                } else {
                    surf_points_extraction_condition =
                        (cloud_neighbor_picked_[ind] == 0 && cloud_curvature_[ind] < params_.surf_th);
                }

                if (surf_points_extraction_condition) {
                    assert(ind >= 0 && ind < cloud_label_.size());
                    cloud_label_[ind] = -1;
                    surf_points_flat_->push_back(segmented_cloud_->points[ind]);

                    smallest_picked_num++;
                    if (smallest_picked_num >= 4) {
                        break;
                    }

                    cloud_neighbor_picked_[ind] = 1;
                    for (int l = 1; l <= 5; l++) {
                        if (ind + l >= seg_info_.segmented_cloud_col_index.size()) break;
                        int column_diff = std::abs(int(seg_info_.segmented_cloud_col_index[ind + l] -
                                                       seg_info_.segmented_cloud_col_index[ind + l - 1]));
                        if (column_diff > 10) break;

                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--) {
                        if (ind + l < 0) break;
                        int column_diff = std::abs(int(seg_info_.segmented_cloud_col_index[ind + l] -
                                                       seg_info_.segmented_cloud_col_index[ind + l + 1]));
                        if (column_diff > 10) break;

                        cloud_neighbor_picked_[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++) {
                if (cloud_label_[k] <= 0) {
                    surf_points_less_flat_scan_->push_back(segmented_cloud_->points[k]);
                }
            }
        }

        surf_points_less_flat_scan_DS_->clear();
        downsize_filter_.setInputCloud(surf_points_less_flat_scan_);
        downsize_filter_.filter(*surf_points_less_flat_scan_DS_);

        *surf_points_less_flat_ += *surf_points_less_flat_scan_DS_;
    }
}

void FeatureExtractor::SetCloudTimeStamp() {
    corner_points_sharp_->header.stamp = segmented_cloud_->header.stamp;
    corner_points_less_sharp_->header.stamp = segmented_cloud_->header.stamp;
    surf_points_flat_->header.stamp = segmented_cloud_->header.stamp;
    surf_points_less_flat_->header.stamp = segmented_cloud_->header.stamp;
}

void FeatureExtractor::Run() {
    PointSegmentation();

    image_projection_->GetSegmentedCloud(segmented_cloud_);
    image_projection_->GetOutlierCloud(outlier_cloud_);
    image_projection_->GetSegInfo(seg_info_);

    AdjustDistortion();
    CalculateSmoothness();
    MarkOccludedPoints();
    ExtractFeatures();
    SetCloudTimeStamp();
}
}  // namespace mapping::core