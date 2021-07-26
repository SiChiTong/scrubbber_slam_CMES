//
// Created by gaoxiang on 2020/9/22.
//

#ifndef MAPPING_LOOP_CLOSING_IMPL_H
#define MAPPING_LOOP_CLOSING_IMPL_H

#include "common/candidate.h"
#include "core/mt_search/mt_search_param.h"
#include "core/mt_search/multi_thread_search.h"

#include <pcl/kdtree/kdtree_flann.h>

namespace mapping::pipeline {

struct LoopClosingImpl {
    /// loop closing 相关
    LoopClosingImpl()
        : kdtree_(new pcl::KdTreeFLANN<pcl::PointXY>()), kfs_2d_cloud_(new pcl::PointCloud<pcl::PointXY>) {}

    core::MTSearchParams params_;

    std::map<IdType, std::vector<common::KFPtr>> keyframes_;      // keyframes by trajectory id
    std::map<IdType, common::KFPtr> keyframes_by_id_;             // keyframes by keyframe id
    std::map<IdType, common::KFPtr> keyframes_by_id_mapping_;     // 建图关键帧
    std::map<IdType, common::KFPtr> keyframes_by_id_validation_;  // 验证关键帧

    std::vector<common::LoopCandidate> loop_candidates_;  // 可能存在的回环

    pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_;       // 用来搜索的kdtree
    pcl::PointCloud<pcl::PointXY>::Ptr kfs_2d_cloud_;  // xy位置组成的点云
    std::map<int, int> kfs_cloud_idx_to_id_;           // 索引点云到id

    std::shared_ptr<core::MultiThreadSearch> mt_search_;  // 多线程搜索器
    bool only_detect_ = false;
    IdType start_id_ = 0;
    IdType end_id_ = 0;

    int gps_status_ = -1;
};

}  // namespace mapping::pipeline
#endif  // MAPPING_LOOP_CLOSING_IMPL_H
