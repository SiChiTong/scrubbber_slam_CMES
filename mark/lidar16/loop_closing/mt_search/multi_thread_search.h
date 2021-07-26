//
// Created by herenjie on 2020/11/10.
//

#ifndef SCRUBBER_SLAM_MULTI_THREAD_SEARCH_H
#define SCRUBBER_SLAM_MULTI_THREAD_SEARCH_H

#include "lidar16/ml_submap.h"
#include "lidar16/ml_tracking.h"
#include "lidar16/ml_frame.h"
#include "mt_search_param.h"
#include "common/num_type.h"
#include "common/candidate.h"
#include "common/mapping_math.h"

#include "thirdparty/ndt_omp/include/pclomp/ndt_omp.h"
#include "common/point_type.h"
#include "../resolution_matching/matcher/multi_resolution_matcher.h"

using namespace mapping::core;
namespace scrubber_slam { namespace lidar16{
    class MTSearchWorker;

    class MultiThreadSearch {
    public:
        MultiThreadSearch(MTSearchParams params,
                          std::shared_ptr<MLTracking> ml_tracker);

        ~MultiThreadSearch(){};

        /**
         * 对外接口：计算回环约束
         * 约束被添加到Keyframes中
         * @param candidates 待验证的candidates
         * @return 接受的回环数量
         */
        int ComputeConstraint(const std::vector<mapping::common::LoopCandidate>& candidates);

        /// 返回计算结果
        std::vector<mapping::common::LoopCandidate> GetResults() const {
            return candidate_results_;
        }

    protected:
        /// 启动匹配线程
        void StartMatching();

   protected:
//    core::MultiResolutionMap::MapRange range_;
        MTSearchParams param_;
//
        std::mutex result_mutex_;
        std::vector<mapping::common::LoopCandidate> candidate_results_;
        std::vector<int> int_vec_test_;
////    std::shared_ptr<core::MemoryControl> mem_control_ = nullptr;
        std::shared_ptr<MLTracking> ml_tracker_;

        std::map<int, std::shared_ptr<MTSearchWorker>> workers_;
    };

    class MTSearchWorker {
    public:
        using FinishCallBack = std::function< void(mapping::common::LoopCandidate candidate)>;
        MTSearchWorker(int index, std::map<IdType, std::shared_ptr<MLFrame>>& keyframes,
                       std::map<Idtype, std::shared_ptr<MLSubmap>> closeloop_map_data,
                       MTSearchParams& param);


    /// 启动子线程
    void Start();

    /// 匹配是否已算完
    bool Finished() { return finished_; }
//
    /// 等待结束
    void Join();
//
//    /// 设置完成时回调函数
    void SetFinishCallBack(FinishCallBack cb) { finish_cb_ = std::move(cb); }

    /// 添加一个待处理的loop candidate
    void AssignCandidateToSearch(mapping::common::LoopCandidate lc);

   private:
    void Reset();

    /// 开始计算
    void Run();

    /// 用node附近的关键帧创建局部点云, world系
    CloudPtr CreateNodeSubmap(const std::shared_ptr<MLFrame> &keyframe, int num );
//    common::PointCloudType::Ptr CreateNodeSubmap(const common::KFPtr& node, int num);

    /// 重置matcher
    void ResetMatcherPtr(double x, double y);

    /// 用NDT计算pose
    /// @return NDT score
    float NDT3D(PointCloudType::Ptr map, PointCloudType::Ptr points, SE3& pose,
                bool save_result = false);

    /// 3D pose 转到2D
    SE3 PoseToGround(const SE3& posea, const SE3& poseb);
//
    /// 为建图搜索回环
    void MatchWithMM();

    /// 为验证轨迹搜索
    void MatchWithNDT();

    int id_ = 0;
    bool finished_;
//    std::atomic<bool> finished_;
    std::thread job_;///一个worker一个独立线程

    std::vector<mapping::common::LoopCandidate> candidates_to_search_;///当前worker分管的回环候选
    std::shared_ptr<MLFrame> kf_a_ = nullptr;//当前帧
    std::shared_ptr<MLFrame> kf_b_ = nullptr;//历史帧
    mapping::common::LoopCandidate current_candidate_;
//
    std::shared_ptr<MLTracking> ml_tracker_;
    std::map<IdType, std::shared_ptr<MLFrame>> keyframes_;
    std::map<Idtype, std::shared_ptr<MLSubmap>> closeloop_map_data_;//回环检测用的submap的ID和submap
    MultiResolutionMap::MapRange range_;
    MTSearchParams& param_;

    std::shared_ptr<MultiResolutionMatcher> multi_matcher_ = nullptr;
////    std::shared_ptr<core::MemoryControl> mem_control_ = nullptr;
//
    FinishCallBack finish_cb_;
//
    double time_used_mm_match_ = 0;
    double time_used_ndt_ = 0;
    };

} }


#endif //SCRUBBER_SLAM_MULTI_THREAD_SEARCH_H
