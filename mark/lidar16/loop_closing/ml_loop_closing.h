//
// Created by herenjie on 2020/11/9.
//

#ifndef SCRUBBER_SLAM_ML_LOOP_CLOSING_H
#define SCRUBBER_SLAM_ML_LOOP_CLOSING_H

//g2o
#include <g2o/core/block_solver.h>
#include <g2o/core/factory.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/types/slam2d/types_slam2d.h>
#include "g2o/types/slam3d/types_slam3d.h"
//ceres
#include <ceres/ceres.h>
#include <ceres/autodiff_cost_function.h>
#include "common/ceres_type.h"

#include <condition_variable>
#include "common/global_config.h"
#include "lidar16/ml_frame.h"
#include "ml_loop_closing_impl.h"

namespace scrubber_slam { namespace lidar16{

    class MLLoopClosing {
    public:
//        MLLoopClosing(std::shared_ptr<MLTracking> ml_tracker);
        MLLoopClosing(MLTracking* ml_tracker);
        ~MLLoopClosing(){
            LOG(INFO)<<"MLLoopClosing deconstruct......................";
//            delete impl_->ml_tracking_;
            impl_->final_loop_candidates_.clear();
        };
        void Run();

        void ComputeRelativeMotionMT();
        void DectectLoop(const std::shared_ptr<MLFrame>& cur_cf);

        void Quit(){
            impl_->quit_request_ = false;
            std::unique_lock<std::mutex>    lm(loopclosure_mutex_);
            loopclosure_cond_.notify_one();
        };

        void PoseOptimization(std::deque<mapping::common::LoopCandidate>& cur_loop_candidates);///by g2o
        void AddVertex(g2o::SparseOptimizer& optimizer,
                       const std::map<Idtype, std::shared_ptr<MLSubmap>>& closeloop_map_data,
                       IdType start_submap_id , IdType end_submap_id);
        void AddLoopClosureFactors(g2o::SparseOptimizer& optimizer,
                                   std::deque<mapping::common::LoopCandidate>& cur_loop_candidates,
                                   int &edge_id);
        void AddSubmapConectFactors(g2o::SparseOptimizer& optimizer,
                                    const std::map<Idtype, std::shared_ptr<MLSubmap>>& closeloop_map_data,
                                    IdType start_submap_id , IdType end_submap_id,
                                    int &edge_id);

//        std::atomic<bool> update_pose_ = false;
        bool update_pose_ = false;
    private:
        std::shared_ptr<MLLoopClosingImpl> impl_;

        std::condition_variable loopclosure_cond_;
        std::mutex    loopclosure_mutex_;
    };

} }


#endif //SCRUBBER_SLAM_ML_LOOP_CLOSING_H
