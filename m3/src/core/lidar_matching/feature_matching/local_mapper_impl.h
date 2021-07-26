//
// Created by gaoxiang on 2020/9/4.
//

#ifndef MAPPING_LOCAL_MAPPER_IMPL_H
#define MAPPING_LOCAL_MAPPER_IMPL_H

#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "core/lidar_matching/feature_matching/matching_param.h"

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <deque>
#include <map>
#include <opencv2/core/core.hpp>

namespace mapping::core {

struct LocalMapperImpl {
    FeatureMatchingParams params_;
    int trajectory_id_;

    float transform_last_[6];
    float transform_sum_[6];
    float transform_incre_[6];
    float transform_tobe_mapped_[6];
    float transform_bef_mapped_[6];
    float transform_aft_mapped_[6];  // pitch, yaw, roll, x, y, z

    SE3 latest_keyframe_pose_;
    bool latest_matching_degenerate_;

    SE3 dr_pose_;

    double fitness_score_;

    bool if_set_height_zero_;

    std::map<int, SE3> matching_poses_;
    std::map<int, V6d> matching_noises_;
    std::vector<bool> matching_degenerate_;
    std::map<int, float> degeneracy_eigenvalue_;

    common::PointCloudType::Ptr laser_cloud_corner_last_;
    common::PointCloudType::Ptr laser_cloud_surf_last_;
    common::PointCloudType::Ptr laser_outlier_cloud_;
    common::PointCloudType::Ptr laser_cloud_surf_total_last_;

    common::PointCloudType::Ptr laser_cloud_corner_last_DS_;
    common::PointCloudType::Ptr laser_cloud_surf_last_DS_;
    common::PointCloudType::Ptr laser_cloud_outlier_last_DS_;
    common::PointCloudType::Ptr laser_cloud_surf_total_last_DS_;

    int latest_frame_ID_;
    std::deque<common::PointCloudType::Ptr> recent_corner_cloud_keyframes_;
    std::deque<common::PointCloudType::Ptr> recent_surf_cloud_keyframes_;
    std::deque<common::PointCloudType::Ptr> recent_outlier_cloud_keyframes_;

    common::PointCloudType::Ptr laser_cloud_corner_from_map_;
    common::PointCloudType::Ptr laser_cloud_surf_from_map_;
    common::PointCloudType::Ptr laser_cloud_corner_from_map_DS_;
    common::PointCloudType::Ptr laser_cloud_surf_from_map_DS_;

    int laser_cloud_corner_from_map_DS_num_;
    int laser_cloud_surf_from_map_DS_num_;
    int laser_cloud_corner_last_DS_num_;
    int laser_cloud_surf_total_last_DS_num_;

    common::PointType previous_robot_pos_point_;
    common::PointType current_robot_pos_point_;

    common::PointCloudType::Ptr cloud_keyposes_3D_;
    pcl::PointCloud<PointTypePose>::Ptr cloud_keyposes_6D_;
    common::PointCloudType::Ptr surrounding_keyposes_;
    common::PointCloudType::Ptr surrounding_keyposes_DS_;

    std::vector<int> point_search_index_;
    std::vector<float> point_search_dis_;

    pcl::KdTreeFLANN<common::PointType>::Ptr kdtree_corner_from_map_;
    pcl::KdTreeFLANN<common::PointType>::Ptr kdtree_surf_from_map_;
    pcl::KdTreeFLANN<common::PointType>::Ptr kdtree_surrounding_keyposes_;

    pcl::VoxelGrid<common::PointType> downsize_filter_corner_;
    pcl::VoxelGrid<common::PointType> downsize_filter_surf_;
    pcl::VoxelGrid<common::PointType> downsize_filter_outlier_;
    pcl::VoxelGrid<common::PointType> downsize_filter_surrounding_keyposes_;

    common::PointCloudType::Ptr laser_cloud_ori_;
    common::PointCloudType::Ptr coeff_sel_;
    common::PointCloudType::Ptr correspond_dis_;

    common::PointType point_ori_, point_sel_, coeff_;

    cv::Mat matA0_;
    cv::Mat matB0_;
    cv::Mat matX0_;

    cv::Mat matA1_;
    cv::Mat matD1_;
    cv::Mat matV1_;

    bool is_degenerate_;
    cv::Mat matP_;

    float cRoll_, sRoll_, cPitch_, sPitch_, cYaw_, sYaw_, tX_, tY_, tZ_;
    float ctRoll_, stRoll_, ctPitch_, stPitch_, ctYaw_, stYaw_, tInX_, tInY_, tInZ_;

    float keyframe_delta_th_;

    int keyframe_id_, previous_id_;
    int start_keyframe_id_;

    std::vector<common::PointCloudType::Ptr> corner_cloud_keyFrames_;
    std::vector<common::PointCloudType::Ptr> surf_cloud_keyFrames_;
    std::vector<common::PointCloudType::Ptr> outlier_cloud_keyFrames_;

    common::PointCloudType::Ptr latest_corner_keyframe_cloud_;
    common::PointCloudType::Ptr latest_surf_keyframe_cloud_;
    common::PointCloudType::Ptr latest_surf_keyframe_cloud_DS_;

    common::PointCloudType::Ptr near_history_surf_keyframe_cloud_;
    common::PointCloudType::Ptr near_history_surf_keyframe_cloud_DS_;

    pcl::KdTreeFLANN<common::PointType>::Ptr kdtree_history_keyposes_;

    pcl::VoxelGrid<common::PointType> downsize_filter_history_keyframes_;

    bool loop_closure_enable_;
    bool potential_loop_flag_;
    int closest_history_frame_ID_;
    int latest_frame_ID_loop_cloure_;
    bool a_loop_is_closed_;
    double loop_current_processing_time_;
    double loop_last_processing_time_;

    std::shared_ptr<g2o::SparseOptimizer> optimizer_;

    g2o::OptimizationAlgorithmLevenberg *solver_;

    int vertex_ID_;
    int edge_ID_;
};
}  // namespace mapping::core
#endif  // MAPPING_LOCAL_MAPPER_IMPL_H
