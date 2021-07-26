//
// Created by herenjie on 2020/11/9.
//

#ifndef SCRUBBER_SLAM_ML_LOOP_CLOSING_IMPL_H
#define SCRUBBER_SLAM_ML_LOOP_CLOSING_IMPL_H

#endif //SCRUBBER_SLAM_ML_LOOP_CLOSING_IMPL_H

#include <pcl/kdtree/kdtree_flann.h>
#include <localization16/ndt_matching/ndt_matching.h>
#include "lidar16/ml_tracking.h"
#include "lidar16/ml_frame.h"
#include "common/candidate.h"

//#include "mt_search/mt_search_param.h"
//#include "mt_search/multi_thread_search.h"

namespace scrubber_slam { namespace lidar16{
    struct MLLoopClosingImpl{
        /// loop closing 相关
        MLLoopClosingImpl()
                : kdtree_(new pcl::KdTreeFLANN<pcl::PointXY>()), kfs_2d_cloud_(new pcl::PointCloud<pcl::PointXY>) {}
//        MTSearchParams params_;

//        std::shared_ptr<MLTracking> ml_tracking_;
        MLTracking* ml_tracking_;
//        std::map<IdType, std::shared_ptr<MLFrame>> keyframes_by_id_;             // keyframes by keyframe id
//        std::map<Idtype, std::shared_ptr<MLSubmap>> closeloop_map_data_;        //回环检测用的submap的ID和submap
        std::mutex loop_canditate_mutex_;
//        std::vector<mapping::common::LoopCandidate> loop_candidates_;          // 可能存在的回环
        std::deque<mapping::common::LoopCandidate> loop_candidates_;          // 可能存在的回环
        std::deque<mapping::common::LoopCandidate> final_loop_candidates_;///最终结果放在这里

            ///当前submap的局部点云地图保存位置
            std::string submap_pcd_path_;



            std::shared_ptr<NdtMatching> ndt_matcher_ = nullptr;

            pcl::KdTreeFLANN<pcl::PointXY>::Ptr kdtree_;       // 用来搜索的kdtree
        pcl::PointCloud<pcl::PointXY>::Ptr kfs_2d_cloud_;  // xy位置组成的点云
        std::map<int, int> kfs_cloud_idx_to_id_;           // 索引点云到id

//        std::atomic<bool> quit_request_ = true;
        bool quit_request_ = true;


//        std::shared_ptr<MultiThreadSearch> mt_search_;  // 多线程搜索器
    };
} }