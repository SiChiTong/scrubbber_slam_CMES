//
// Created by gaoxiang on 2020/9/16.
//

#ifndef MAPPING_OPTIMIZATION_S1_IMPL_H
#define MAPPING_OPTIMIZATION_S1_IMPL_H

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include "common/gps_status_def.h"
#include "common/keyframe.h"
#include "common/num_type.h"
#include "common/origin_point_info.h"
#include "core/optimization/edge_se3_height_prior.h"
#include "core/optimization/edge_se3_rot_prior.h"
#include "pipeline/optimization_s1/optimization_params.h"

namespace mapping::pipeline {

struct OptimizationStage1Impl {
    OptimizationStage1Impl() = default;
    ~OptimizationStage1Impl() = default;
    OptimizationParams optimization_params_;
    common::OriginPointInformation origin_info_;
    common::BagGNSSStatusType gps_status_;

    std::map<IdType, std::vector<common::KFPtr>> keyframes_;
    std::map<IdType, common::KFPtr> keyframes_map_;

    bool if_save_pcd_ = false;
    bool if_merge_maps_ = false;
    bool if_save_floor_ = true;
    bool is_open_calib_lidar_gnss_ = true;

    // optimizer related
    int edge_id_ = 0;
    double gps_th_;

    g2o::SparseOptimizer optimizer_;
    std::vector<g2o::VertexSE3 *> vertices_;                // pose graph顶点
    std::vector<g2o::EdgeSE3 *> matching_edges_;            // 激光边
    std::vector<g2o::EdgeSE3 *> dr_edges_;                  // Dr边
    std::vector<g2o::EdgeSE3Prior *> gps_edges_;            // GPS边
    std::vector<g2o::EdgeSE3Prior *> manual_edges_;         // 人工回环边
    std::vector<core::EdgeSE3HeightPrior *> height_edges_;  // 高度
    std::vector<core::EdgeSE3RotPrior *> rot_edges_;        // 全局旋转
    std::vector<g2o::EdgeSE3 *> loop_edges_;                // 回环边

    std::vector<std::pair<int, int>> loop_edges_index_;

    SE3 TGL_ = SE3();  // 外参：GNSS至Lidar
    float gps_inlier_ratio_ = 0.0;

    std::vector<common::LoopCandidate> loop_candidates_;  // 可能存在的轨迹连接点

    bool save_debug_pcd_ = false;
    bool if_use_nav_sat_ = true;

    bool valid_bag_use_matching_pose_ = true;
};

}  // namespace mapping::pipeline
#endif  // MAPPING_OPTIMIZATION_S1_IMPL_H
