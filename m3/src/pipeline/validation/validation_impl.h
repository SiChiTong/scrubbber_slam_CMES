//
// Created by gaoxiang on 2020/10/22.
//

#ifndef MAPPING_VALIDATION_IMPL_H
#define MAPPING_VALIDATION_IMPL_H

#include "common/car_type.h"
#include "common/keyframe.h"
#include "common/num_type.h"

#include "core/mt_search/multi_thread_search.h"
#include "core/optimization/edge_se3_height_prior.h"

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <src/pipeline/optimization_s1/optimization_params.h>

namespace mapping::pipeline {

struct ValidationImpl {
    ValidationImpl()
        : kdtree_(new pcl::KdTreeFLANN<pcl::PointXY>()), kfs_2d_cloud_(new pcl::PointCloud<pcl::PointXY>) {}

    common::CarType car_type_ = common::CarType::WXX;
    std::map<IdType, std::vector<common::KFPtr>> keyframes_;          // 按轨迹id排序
    std::map<IdType, common::KFPtr> keyframes_map_by_id_;             // 按kf id排序
    std::map<IdType, common::KFPtr> keyframes_map_by_id_validation_;  // validation
    std::map<IdType, common::KFPtr> keyframes_map_by_id_mapping_;     // mapping

    core::MTSearchParams params_;
    std::shared_ptr<core::MultiThreadSearch> mt_search_ = nullptr;

    int gps_status_ = -1;

    /// KD树相关
    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_;       // 用来搜索的kdtree
    pcl::PointCloud<pcl::PointXY>::Ptr kfs_2d_cloud_;  // xy位置组成的点云
    std::map<int, int> kfs_cloud_idx_to_id_;           // 索引点云到id

    std::vector<common::LoopCandidate> loop_candidates_gnss_search_;
    std::vector<common::LoopCandidate> loop_candidates_gnss_search_result_;

    std::vector<common::LoopCandidate> loop_candidates_smoothing_search_;
    std::vector<common::LoopCandidate> loop_candidates_smoothing_search_result_;

    std::vector<common::LoopCandidate> loop_candidates_exhaustive_search_;
    std::vector<common::LoopCandidate> loop_candidates_exhaustive_search_result_;

    std::vector<common::LoopCandidate> loops_score_result_;

    std::map<common::KFPtr, int> kf_localized_map_;  // 被正确定位的关键帧数量

    OptimizationParams optimization_params_;
    g2o::SparseOptimizer optimizer_;

    int edge_id_ = 0;
    std::vector<g2o::VertexSE3 *> vertices_;               // pose graph顶点
    std::map<IdType, g2o::VertexSE3 *> kfid_to_vertices_;  // 从关键帧ID索引g2o顶点

    std::vector<g2o::EdgeSE3 *> dr_edges_;                  // Dr边
    std::vector<g2o::EdgeSE3 *> matching_edges_;            // matching 边
    std::vector<g2o::EdgeSE3 *> loop_edges_;                // 回环边
    std::vector<core::EdgeSE3HeightPrior *> height_edges_;  // 高度

    double average_height_ = 0.0;

    double loop_exhaustive_rk_factor_ = 100.0;
};

}  // namespace mapping::pipeline

#endif  // MAPPING_VALIDATION_IMPL_H
