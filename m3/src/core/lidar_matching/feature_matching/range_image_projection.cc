//
// Created by wangqi on 19-7-19.
//

#include "core/lidar_matching/feature_matching/range_image_projection.h"
#include "common/constants.h"

namespace mapping::core {

using namespace mapping::common;

ImageProjection::ImageProjection(FeatureMatchingParams& params) : params_(params) {
    nan_point_.x = std::numeric_limits<float>::quiet_NaN();
    nan_point_.y = std::numeric_limits<float>::quiet_NaN();
    nan_point_.z = std::numeric_limits<float>::quiet_NaN();
    nan_point_.range = -1;
    nan_point_.intensity = -1;

    input_laser_cloud_.reset(new PointCloudType());
    segmented_cloud_.reset(new PointCloudType());
    outlier_cloud_.reset(new PointCloudType());

    full_cloud_.reset(new PointCloudType());
    full_info_cloud_.reset(new PointCloudType());

    full_cloud_->points.resize(16 * 1800);
    full_info_cloud_->points.resize(16 * 1800);
    std::fill(full_cloud_->points.begin(), full_cloud_->points.end(), nan_point_);
    std::fill(full_info_cloud_->points.begin(), full_info_cloud_->points.end(), nan_point_);

    seg_msg_.start_ring_index.assign(16, 0);
    seg_msg_.end_ring_index.assign(16, 0);
    seg_msg_.segmented_cloud_ground_flag.assign(16 * 1800, false);
    seg_msg_.segmented_cloud_col_index.assign(16 * 1800, 0);
    seg_msg_.segmented_cloud_range.assign(16 * 1800, 0);

    std::pair<int8_t, int8_t> neighbor;
    neighbor.first = -1;
    neighbor.second = 0;
    neighbor_iterator_.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = 1;
    neighbor_iterator_.push_back(neighbor);
    neighbor.first = 0;
    neighbor.second = -1;
    neighbor_iterator_.push_back(neighbor);
    neighbor.first = 1;
    neighbor.second = 0;
    neighbor_iterator_.push_back(neighbor);

    all_pushed_ind_x_.resize(16 * 1800);
    all_pushed_ind_y_.resize(16 * 1800);

    queue_ind_x_.resize(16 * 1800);
    queue_ind_y_.resize(16 * 1800);

    range_mat_ = cv::Mat(16, 1800, CV_32F, cv::Scalar::all(FLT_MAX));
    ground_mat_ = cv::Mat(16, 1800, CV_8S, cv::Scalar::all(0));
    label_mat_ = cv::Mat(16, 1800, CV_32S, cv::Scalar::all(0));
    label_count_ = 1;

    cloud_z_offset_ = params_.cloud_z_offset;
}

ImageProjection::~ImageProjection() {}

void ImageProjection::ImageProjection::ResetParameters() {
    segmented_cloud_->clear();
    outlier_cloud_->clear();

    range_mat_ = cv::Mat(16, 1800, CV_32F, cv::Scalar::all(FLT_MAX));
    ground_mat_ = cv::Mat(16, 1800, CV_8S, cv::Scalar::all(0));
    label_mat_ = cv::Mat(16, 1800, CV_32S, cv::Scalar::all(0));
    label_count_ = 1;

    std::fill(full_cloud_->points.begin(), full_cloud_->points.end(), nan_point_);
    std::fill(full_info_cloud_->points.begin(), full_info_cloud_->points.end(), nan_point_);
}

void ImageProjection::FindStartEndAngle() {
    seg_msg_.start_orientation = -atan2(input_laser_cloud_->points[0].y, input_laser_cloud_->points[0].x);
    seg_msg_.end_orientation = -atan2(input_laser_cloud_->points[input_laser_cloud_->points.size() - 1].y,
                                      input_laser_cloud_->points[input_laser_cloud_->points.size() - 2].x) +
                               2 * kPI;

    if (seg_msg_.end_orientation - seg_msg_.start_orientation > 3 * kPI) {
        seg_msg_.end_orientation -= 2 * kPI;
    } else if (seg_msg_.end_orientation - seg_msg_.start_orientation < kPI)
        seg_msg_.end_orientation += 2 * kPI;
    seg_msg_.orientation_diff = seg_msg_.end_orientation - seg_msg_.start_orientation;
}

void ImageProjection::ProjectPointCloud() {
    float vertical_angle, horizon_angle, range;
    int row_index, column_index, index, cloud_size;
    common::PointType this_point;

    cloud_size = input_laser_cloud_->points.size();

    for (int i = 0; i < cloud_size; ++i) {
        this_point.x = input_laser_cloud_->points[i].x;
        this_point.y = input_laser_cloud_->points[i].y;
        this_point.z = input_laser_cloud_->points[i].z - cloud_z_offset_;

        vertical_angle =
            atan2(this_point.z, sqrt(this_point.x * this_point.x + this_point.y * this_point.y)) * 180 / kPI;
        row_index = (vertical_angle + params_.ang_bottom) / params_.ang_res_y;
        if (row_index < 0 || row_index >= 16) continue;

        horizon_angle = atan2(this_point.x, this_point.y) * 180 / kPI;

        if (horizon_angle <= -90) column_index = -int(horizon_angle / params_.ang_res_x) - 450;
        else if (horizon_angle >= 0)
            column_index = -int(horizon_angle / params_.ang_res_x) + 1350;
        else
            column_index = 1350 - int(horizon_angle / params_.ang_res_x);

        range = sqrt(this_point.x * this_point.x + this_point.y * this_point.y + this_point.z * this_point.z);
        range_mat_.at<float>(row_index, column_index) = range;

        this_point.range = (float)row_index + (float)column_index / 10000.0;
        this_point.intensity = (float)row_index + (float)column_index / 10000.0;

        index = column_index + row_index * 1800;
        full_cloud_->points[index] = this_point;
        full_info_cloud_->points[index].range = range;
        full_info_cloud_->points[index].intensity = range;
    }
}

void ImageProjection::GroundRemoval() {
    int lower_index, upper_index;
    float diff_x, diff_y, diff_z, angle;

    for (int j = 0; j < 1800; ++j) {
        for (int i = 0; i < params_.ground_scan_index; ++i) {
            lower_index = j + (i)*1800;
            upper_index = j + (i + 1) * 1800;

            if (full_cloud_->points[lower_index].range == -1 || full_cloud_->points[upper_index].range == -1) {
                ground_mat_.at<int8_t>(i, j) = -1;
                continue;
            }

            diff_x = full_cloud_->points[upper_index].x - full_cloud_->points[lower_index].x;
            diff_y = full_cloud_->points[upper_index].y - full_cloud_->points[lower_index].y;
            diff_z = full_cloud_->points[upper_index].z - full_cloud_->points[lower_index].z;

            angle = atan2(diff_z, sqrt(diff_x * diff_x + diff_y * diff_y)) * 180 / M_PI;

            if (angle <= 10 && angle >= -10) {
                ground_mat_.at<int8_t>(i, j) = 1;
                ground_mat_.at<int8_t>(i + 1, j) = 1;
            }
        }
    }

    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 1800; ++j) {
            if (ground_mat_.at<int8_t>(i, j) == 1 || range_mat_.at<float>(i, j) == FLT_MAX) {
                label_mat_.at<int>(i, j) = -1;
            }
        }
    }
}

void ImageProjection::CloudSegmentation() {
    for (int i = 0; i < 16; ++i) {
        for (int j = 0; j < 1800; ++j) {
            if (label_mat_.at<int>(i, j) == 0) {
                LabelComponents(i, j);
            }
        }
    }

    int sizeOfSegCloud = 0;
    for (int i = 0; i < 16; ++i) {
        seg_msg_.start_ring_index[i] = sizeOfSegCloud - 1 + 5;

        for (int j = 0; j < 1800; ++j) {
            if (params_.segment_enabled) {
                if (label_mat_.at<int>(i, j) > 0 || ground_mat_.at<int8_t>(i, j) == 1) {
                    if (label_mat_.at<int>(i, j) == 999999) {
                        if (i > params_.ground_scan_index && j % 5 == 0) {
                            outlier_cloud_->push_back(full_cloud_->points[j + i * 1800]);
                            continue;
                        } else {
                            continue;
                        }
                    }
                    if (ground_mat_.at<int8_t>(i, j) == 1) {
                        if (j % 5 != 0 && j > 5 && j < 1800 - 5) continue;
                    }
                    seg_msg_.segmented_cloud_ground_flag[sizeOfSegCloud] = (ground_mat_.at<int8_t>(i, j) == 1);
                    seg_msg_.segmented_cloud_col_index[sizeOfSegCloud] = j;
                    seg_msg_.segmented_cloud_range[sizeOfSegCloud] = range_mat_.at<float>(i, j);
                    segmented_cloud_->push_back(full_cloud_->points[j + i * 1800]);
                    ++sizeOfSegCloud;
                }
            } else {
                common::PointType single_point = full_cloud_->points[j + i * 1800];
                if (single_point.range != -1) {
                    seg_msg_.segmented_cloud_ground_flag[sizeOfSegCloud] = false;
                    seg_msg_.segmented_cloud_col_index[sizeOfSegCloud] = j;
                    seg_msg_.segmented_cloud_range[sizeOfSegCloud] = range_mat_.at<float>(i, j);
                    segmented_cloud_->push_back(single_point);
                    ++sizeOfSegCloud;
                }
            }
        }

        seg_msg_.end_ring_index[i] = sizeOfSegCloud - 1 - 5;
    }
}

void ImageProjection::LabelComponents(int row, int col) {
    float d1, d2, alpha, angle;
    int from_ind_x, from_ind_y, this_ind_x, this_ind_y;
    bool line_count_flag[16] = {false};

    queue_ind_x_[0] = row;
    queue_ind_y_[0] = col;
    int queue_size = 1;
    int queue_start_index = 0;
    int queue_end_index = 1;

    all_pushed_ind_x_[0] = row;
    all_pushed_ind_y_[0] = col;
    int all_pushed_index_size = 1;

    while (queue_size > 0) {
        from_ind_x = queue_ind_x_[queue_start_index];
        from_ind_y = queue_ind_y_[queue_start_index];
        --queue_size;
        ++queue_start_index;
        label_mat_.at<int>(from_ind_x, from_ind_y) = label_count_;

        for (auto& iter : neighbor_iterator_) {
            this_ind_x = from_ind_x + iter.first;
            this_ind_y = from_ind_y + iter.second;

            if (this_ind_x < 0 || this_ind_x >= 16) continue;

            if (this_ind_y < 0) this_ind_y = 1800 - 1;
            if (this_ind_y >= 1800) this_ind_y = 0;

            if (label_mat_.at<int>(this_ind_x, this_ind_y) != 0) continue;

            d1 = std::max(range_mat_.at<float>(from_ind_x, from_ind_y), range_mat_.at<float>(this_ind_x, this_ind_y));
            d2 = std::min(range_mat_.at<float>(from_ind_x, from_ind_y), range_mat_.at<float>(this_ind_x, this_ind_y));

            if (iter.first == 0) {
                alpha = params_.segment_alpha_x;
            } else {
                alpha = params_.segment_alpha_y;
            }

            angle = atan2(d2 * sin(alpha), (d1 - d2 * cos(alpha)));

            if (angle > 1.0472) {
                queue_ind_x_[queue_end_index] = this_ind_x;
                queue_ind_y_[queue_end_index] = this_ind_y;
                ++queue_size;
                ++queue_end_index;

                label_mat_.at<int>(this_ind_x, this_ind_y) = label_count_;
                line_count_flag[this_ind_x] = true;

                all_pushed_ind_x_[all_pushed_index_size] = this_ind_x;
                all_pushed_ind_y_[all_pushed_index_size] = this_ind_y;
                ++all_pushed_index_size;
            }
        }
    }

    bool feasible_segment = false;
    if (all_pushed_index_size >= 30) {
        feasible_segment = true;
    } else if (all_pushed_index_size >= 5) {
        int line_count = 0;
        for (bool i : line_count_flag)
            if (i) {
                ++line_count;
            }
        if (line_count >= 3) {
            feasible_segment = true;
        }
    }

    if (feasible_segment) {
        ++label_count_;
    } else {
        for (int i = 0; i < all_pushed_index_size; ++i) {
            label_mat_.at<int>(all_pushed_ind_x_[i], all_pushed_ind_y_[i]) = 999999;
        }
    }
}

void ImageProjection::SetCloudTimeStamp() {
    segmented_cloud_->header.stamp = input_laser_cloud_->header.stamp;
    outlier_cloud_->header.stamp = input_laser_cloud_->header.stamp;
}

void ImageProjection::CloudHandler() {
    ResetParameters();
    FindStartEndAngle();
    ProjectPointCloud();
    GroundRemoval();
    CloudSegmentation();
    SetCloudTimeStamp();
}

}  // namespace mapping::core