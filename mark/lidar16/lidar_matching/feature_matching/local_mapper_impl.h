//
// Created by gaoxiang on 2020/9/4.
//

#ifndef MAPPING_LOCAL_MAPPER_IMPL_H
#define MAPPING_LOCAL_MAPPER_IMPL_H

#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "lidar16/lidar_matching/feature_matching/matching_param.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <deque>
#include <map>
#include <opencv2/core/core.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include "common/ceres_type.h"

namespace mapping {
namespace core {

struct LocalMapperImpl {
    FeatureMatchingParams params_;

    std::vector<bool> matching_degenerate_;

    ///当前帧的点转到世界坐标系的变换，每次迭代优化求解都会更新，初始值为开始传入的值
    Eigen::Quaterniond q_opt_;
    Eigen::Vector3d t_opt_;

    Eigen::Quaterniond q_prior_;
    Eigen::Vector3d t_prior_;

    CloudPtr laser_cloud_corner_last_;  ///上一帧的角点
    CloudPtr laser_cloud_surf_last_;    ///上一帧的平面点
    CloudPtr laser_outlier_cloud_;      ///上一帧的outlier点
    CloudPtr laser_cloud_surf_total_last_;

    CloudPtr laser_cloud_corner_last_DS_;  ///当前帧角点
    CloudPtr laser_cloud_surf_last_DS_;
    CloudPtr laser_cloud_surf_total_last_DS_;  ///当前帧平面点
    CloudPtr ground_cloud_cur_scan_DS_;

    CloudPtr laser_cloud_corner_from_map_;
    CloudPtr laser_cloud_surf_from_map_;
    CloudPtr laser_cloud_corner_from_map_DS_;  ///局部地图角点,世界坐标系下
    CloudPtr laser_cloud_surf_from_map_DS_;
    CloudPtr ground_cloud_local_map_DS_;

    int laser_cloud_corner_from_map_DS_num_;  ///局部帧的角点数量
    int laser_cloud_surf_from_map_DS_num_;    ///局部帧的平面点数量
    int ground_cloud_local_map_DS_num_;       ///局部帧的平面点数量

    int laser_cloud_corner_last_DS_num_;      ///当前帧的角点数量
    int laser_cloud_surf_total_last_DS_num_;  ///当前帧的平面点数量
    int ground_cloud_cur_scan_DS_num_;  ///当前帧的平面点数量

    std::vector<int> point_search_index_;
    std::vector<float> point_search_dis_;

    pcl::KdTreeFLANN<PointType>::Ptr kdtree_corner_from_map_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_surf_from_map_;
    pcl::KdTreeFLANN<PointType>::Ptr kdtree_ground_from_map_;

    pcl::VoxelGrid<PointType> downsize_filter_corner_;
    pcl::VoxelGrid<PointType> downsize_filter_surf_;
    pcl::VoxelGrid<PointType> downsize_filter_ground_;

    CloudPtr laser_cloud_ori_;
    CloudPtr coeff_sel_;
    CloudPtr correspond_dis_;

    PointType point_ori_, point_sel_, coeff_;

    cv::Mat matA0_;
    cv::Mat matB0_;
    cv::Mat matX0_;

    cv::Mat matA1_;  ///局部地图中的轮廓点直线的协方差
    cv::Mat matD1_;
    cv::Mat matV1_;
};
}  // namespace core
}  // namespace mapping
#endif  // MAPPING_LOCAL_MAPPER_IMPL_H
