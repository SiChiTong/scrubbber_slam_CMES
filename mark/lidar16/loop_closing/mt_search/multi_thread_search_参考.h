//
// Created by gaoxiang on 2020/9/22.
//

#ifndef MAPPING_MULTI_THREAD_SEARCH_H
#define MAPPING_MULTI_THREAD_SEARCH_H

#include "common/num_type.h"
#include "mt_search_param.h"
#include "../ml_loop_closing_impl.h"
#include "lidar16/ml_frame.h"
//#include "core/resolution_matching/matcher/multi_resolution_map.h"


#include <atomic>
#include <mutex>
#include <thread>

namespace scrubber_slam::lidar16 {
//class MemoryControl;
//class MultiResolutionMatcher;
    class MTSearchWorker;

/**
 * MultiThreadSearch
 * 多线程回环检测算法，给定由距离检测出来的回环候选，用每个候选创建一个worker，然后用多线程方式管理worker的执行
 * 每一个worker执行完毕后，如果回环成功检出，向该类返回一个检测结果
 * */
    class MultiThreadSearch {
    public:

        MultiThreadSearch(const std::string& local_data_path, MTSearchParams params,
                          std::map<IdType, std::shared_ptr<MLFrame>>& keyframes_by_id);

//    /**
//     * 对外接口：计算回环约束
//     * 约束被添加到Keyframes中
//     * @param candidates 待验证的candidates
//     * @return 接受的回环数量
//     */
//    int ComputeConstraint(const std::vector<LoopCandidate>& candidates);
//
//    /// 返回计算结果
//    std::vector<LoopCandidate> GetResults() const { return candidate_results_; }
//
//   protected:
//    /// 启动匹配线程
//    void StartMatching();
//
//   protected:
//    core::MultiResolutionMap::MapRange range_;
//    MTSearchParams param_;
//
//    std::mutex result_mutex_;
//    std::vector<LoopCandidate> candidate_results_;
////    std::shared_ptr<core::MemoryControl> mem_control_ = nullptr;
//    std::map<IdType, std::shared_ptr<MLFrame>>& keyframes_by_id_;
//
//    std::map<int, std::shared_ptr<MTSearchWorker>> workers_;
    };

/**
 * 执行器
 * 构建multi resolution map，计算score，然后用NDT验证
 */
//class MTSearchWorker {
//   public:
//    using FinishCallBack = std::function<void(common::LoopCandidate candidate)>;
//
//    /// constructor填入ID，关键帧和回环参数
////    MTSearchWorker(int index, std::map<IdType, common::KFPtr>& keyframes, core::MultiResolutionMap::MapRange& range,
////                   MTSearchParams& param, std::shared_ptr<core::MemoryControl> mem_control = nullptr);
//    MTSearchWorker(int index, std::map<IdType, std::shared_ptr<MLFrame>>& keyframes, core::MultiResolutionMap::MapRange& range,
//                   MTSearchParams& param);
//
//    /// 启动子线程
//    void Start();
//
//    /// 匹配是否已算完
//    bool Finished() { return finished_; }
//
//    /// 等待结束
//    void Join();
//
//    /// 设置完成时回调函数
//    void SetFinishCallBack(FinishCallBack cb) { finish_cb_ = std::move(cb); }
//
//    /// 添加一个待处理的loop candidate
//    void AssignCandidateToSearch(LoopCandidate lc);
//
//   private:
//    void Reset();
//
//    /// 开始计算
//    void Run();
//
//    /// 用node附近的关键帧创建局部点云, world系
////    common::PointCloudType::Ptr CreateNodeSubmap(const common::KFPtr& node, int num);
//
//    /// 重置matcher
//    void ResetMatcherPtr(double x, double y);
//
//    /// 用NDT计算pose
//    /// @return NDT score
//    float NDT3D(common::PointCloudType::Ptr map, common::PointCloudType::Ptr points, SE3& pose,
//                bool save_result = false);
//
//    /// 3D pose 转到2D
//    SE3 PoseToGround(const SE3& posea, const SE3& poseb);
//
//    /// 为建图搜索回环
//    void MatchWithMM();
//
//    /// 为验证轨迹搜索
//    void MatchWithNDT();
//
//    int id_ = 0;
//    std::atomic<bool> finished_;
//    std::thread job_;
//
//    std::vector<LoopCandidate> candidates_to_search_;
//    std::shared_ptr<MLFrame> kf_a_ = nullptr;
//    std::shared_ptr<MLFrame> kf_b_ = nullptr;
//    LoopCandidate current_candidate_;
//
//    std::map<IdType, std::shared_ptr<MLFrame>>& keyframes_;
//    core::MultiResolutionMap::MapRange range_;
//    MTSearchParams& param_;
//
//    std::shared_ptr<core::MultiResolutionMatcher> multi_matcher_ = nullptr;
////    std::shared_ptr<core::MemoryControl> mem_control_ = nullptr;
//
//    FinishCallBack finish_cb_;
//
//    double time_used_mm_match_ = 0;
//    double time_used_ndt_ = 0;
//};

}  // namespace mapping::core

#endif  // MAPPING_MULTI_THREAD_SEARCH_H
