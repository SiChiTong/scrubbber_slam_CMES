//
// Created by gaoxiang on 2020/9/22.
//

#ifndef MAPPING_MULTI_THREAD_SEARCH_H
#define MAPPING_MULTI_THREAD_SEARCH_H

#include "common/keyframe.h"
#include "common/num_type.h"
#include "core/mt_search/mt_search_param.h"
#include "core/resolution_matching/matcher/multi_resolution_map.h"
#include "src/common/candidate.h"

#include <atomic>
#include <mutex>
#include <thread>

namespace mapping::core {
class MemoryControl;
class MultiResolutionMatcher;
class MTSearchWorker;

/**
 * MultiThreadSearch
 * 多线程回环检测算法，给定由距离检测出来的回环候选，用每个候选创建一个worker，然后用多线程方式管理worker的执行
 * 每一个worker执行完毕后，如果回环成功检出，向该类返回一个检测结果
 * */
class MultiThreadSearch {
   public:
    MultiThreadSearch(const std::string& local_db_path, MTSearchParams params,
                      std::map<IdType, common::KFPtr>& keyframes_by_id, bool run_as_gem = false);

    /**
     * 对外接口：计算回环约束
     * 约束被添加到Keyframes中
     * @param candidates 待验证的candidates
     * @return 接受的回环数量
     */
    int ComputeConstraint(const std::vector<common::LoopCandidate>& candidates);

    /// 返回计算结果
    std::vector<common::LoopCandidate> GetResults() const { return candidate_results_; }

    MTSearchParams GetMTSearchParams() { return param_; }

   protected:
    /// 启动匹配线程
    void StartMatching();

   protected:
    core::MultiResolutionMap::MapRange range_;
    MTSearchParams param_;

    std::mutex result_mutex_;
    std::vector<common::LoopCandidate> candidate_results_;
    std::shared_ptr<core::MemoryControl> mem_control_ = nullptr;
    std::map<IdType, common::KFPtr>& keyframes_by_id_;

    std::map<int, std::shared_ptr<MTSearchWorker>> workers_;

   private:
    double transformation_epsilon_ = 0;
};

/**
 * 执行器
 * 构建multi resolution map，计算score，然后用NDT验证
 */
class MTSearchWorker {
   public:
    using FinishCallBack = std::function<void(common::LoopCandidate candidate)>;

    /// constructor填入ID，关键帧和回环参数
    MTSearchWorker(int index, std::map<IdType, common::KFPtr>& keyframes, core::MultiResolutionMap::MapRange& range,
                   MTSearchParams& param, std::shared_ptr<core::MemoryControl> mem_control = nullptr);

    /// 启动子线程
    void Start();

    /// 匹配是否已算完
    bool Finished() { return finished_; }

    /// 等待结束
    void Join();

    /// 设置完成时回调函数
    void SetFinishCallBack(FinishCallBack cb) { finish_cb_ = std::move(cb); }

    /// 添加一个待处理的loop candidate
    void AssignCandidateToSearch(common::LoopCandidate lc);

    void Init() {}

    void SetParam(const MTSearchParams& params) { param_ = params; }

   private:
    void Reset();

    /// 开始计算
    void Run();

    /// 用node附近的关键帧创建局部点云, world系
    common::PointCloudType::Ptr CreateNodeSubmap(const common::KFPtr& node, int num);

    /// 重置matcher
    void ResetMatcherPtr(double x, double y);

    /// 用NDT计算pose
    /// @return NDT score
    float NDT3D(common::PointCloudType::Ptr map, common::PointCloudType::Ptr points, SE3& pose,
                bool save_result = false);

    double ICP(common::PointCloudType::Ptr map, common::PointCloudType::Ptr points, M4d& pose,
               bool save_result = false);

    /// 3D pose 转到2D
    SE3 PoseToGround(const SE3& posea, const SE3& poseb);

    /// 为建图搜索回环
    void MatchWithMM();

    /// 为验证轨迹搜索
    void MatchWithNDT();

    void MatchWithICP();

    static double CalcFitnessScore(const common::PointCloudType::ConstPtr& cloud1,
                                   const common::PointCloudType::ConstPtr& cloud2, const Eigen::Isometry3d& relpose,
                                   double max_range = std::numeric_limits<double>::max());

    int id_ = 0;
    std::atomic<bool> finished_;
    std::thread job_;

    std::vector<common::LoopCandidate> candidates_to_search_;
    common::KFPtr kf_a_ = nullptr;
    common::KFPtr kf_b_ = nullptr;
    common::LoopCandidate current_candidate_;

    std::map<IdType, common::KFPtr>& keyframes_;
    core::MultiResolutionMap::MapRange range_;
    MTSearchParams& param_;

    std::shared_ptr<core::MultiResolutionMatcher> multi_matcher_ = nullptr;
    std::shared_ptr<core::MemoryControl> mem_control_ = nullptr;

    FinishCallBack finish_cb_;

    double time_used_mm_match_ = 0;
    double time_used_ndt_ = 0;

    double transformation_epsilon_ = 0.01;

    bool ndt_map_remove_ground_ = false;
};

}  // namespace mapping::core

#endif  // MAPPING_MULTI_THREAD_SEARCH_H
