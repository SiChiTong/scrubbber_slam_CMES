//
// Created by herenjie on 2020/11/9.
//

#include <types/slam3d/vertex_se3.h>
#include <types/slam3d/edge_se3.h>
#include "ml_loop_closing.h"

namespace scrubber_slam { namespace lidar16{
using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
inline g2o::SE3Quat SE3ToG2OSE3Quat(const SE3 &T) {
    return g2o::SE3Quat(T.so3().matrix(), T.translation());
}

//MLLoopClosing::MLLoopClosing(std::shared_ptr<MLTracking> ml_tracker)
MLLoopClosing::MLLoopClosing(MLTracking* ml_tracker)
:impl_(new MLLoopClosingImpl)
{
    auto yaml = GlobalConfig::Get();
//    impl_->params_.LoadFromYAML(*yaml);
            impl_->submap_pcd_path_ = yaml->GetValue<std::string>("map_saver_param", "submap_pcd_path");
    impl_->ml_tracking_ = ml_tracker;

//    impl_->mt_search_ =
//            std::make_shared<MultiThreadSearch>(impl_->params_, impl_->ml_tracking_);

    impl_->loop_candidates_.clear();

}

void MLLoopClosing::Run(){
    while (impl_->quit_request_)
    {
        {
            std::unique_lock<std::mutex>    lm(loopclosure_mutex_);
                loopclosure_cond_.wait(lm);
                sleep(1);
        }
        if(!impl_->quit_request_)
            return;

        LOG(ERROR)<<"MLLoopClosing::Run";

        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        ComputeRelativeMotionMT();
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0);
        LOG(INFO) << "NDT回环检测用时：" << time_used.count() << " 秒。" << endl;
    }
}

void MLLoopClosing::ComputeRelativeMotionMT() {
    std::deque<mapping::common::LoopCandidate> cur_loop_candidates;
    {
        std::unique_lock<std::mutex> lcm(impl_->loop_canditate_mutex_);
        cur_loop_candidates = impl_->loop_candidates_;
        impl_->loop_candidates_.clear();
    }

    pclomp::NormalDistributionsTransform<PointType, PointType> ndt_matcher;
//    pcl::NormalDistributionsTransform<PointType, PointType> ndt_matcher;
    ndt_matcher.setResolution(1);
    ndt_matcher.setStepSize(0.01);
    ndt_matcher.setTransformationEpsilon(0.01);
    ndt_matcher.setMaximumIterations(200);
//    ndt_matcher.setMaximumIterations(50);
//    ndt_matcher.setNumThreads(1);


    auto keyframs_by_id = impl_->ml_tracking_->GetKeyframByID();///所有的关键帧
    auto submaps_by_id = impl_->ml_tracking_->GetAllSubMap();///所有的submaps
    LOG(INFO)<<"当前次要做ndt 次数： "<<cur_loop_candidates.size();
    std::vector<int> kf_id2clc_vector;
    for(auto &lc: cur_loop_candidates){
        CloudPtr cloud_submap(new PointCloudType);

//        *cloud_submap = *submaps_by_id[lc.smid]->GetCloud();///submap的局部点云在其局部坐标系下
        pcl::io::loadPCDFile(impl_->submap_pcd_path_+ "submap_" + std::to_string(lc.smid) + ".pcd", *cloud_submap);///submap的局部点云在其局部坐标系下
        SE3 T_l2submap = keyframs_by_id[lc.kfid]->Tsl_;
        CloudPtr cloud_kf_submap(new PointCloudType);///关键帧的点云在其submap坐标系
        pcl::transformPointCloud(*keyframs_by_id[lc.kfid]->cloud_ptr_, *cloud_kf_submap,T_l2submap.matrix());

        bool has = false;
        for(auto &kc: kf_id2clc_vector){
            if(kc == lc.kfid){
                has = true; break;
            }
        }
        if(!has)  kf_id2clc_vector.emplace_back(lc.kfid);

        SE3 input_guess = submaps_by_id[lc.smid]->GetLocalPose().inverse() *
                          submaps_by_id[keyframs_by_id[lc.kfid]->kf_in_submap_id_]->GetLocalPose();
/*
        auto RemoveGroundCeiling = [=](const CloudPtr& input_cloud,CloudPtr& output_cloud, const float z_g, const float z_c){
            for(auto& pt: *input_cloud){
                if(pt.z > z_g && pt.z < z_c )
                    output_cloud->push_back(pt);
            }
            return output_cloud;
        };

        CloudPtr cloud_kf_submap_rg(new PointCloudType);///关键帧的点云在其submap坐标系
        CloudPtr cloud_submap_rg(new PointCloudType);///关键帧的点云在其submap坐标系
        RemoveGroundCeiling(cloud_submap, cloud_submap_rg,0.3+1.32,3.0+1.32);
        RemoveGroundCeiling(cloud_kf_submap, cloud_kf_submap_rg,0.3,3.0);
        ndt_matcher.setInputSource(cloud_kf_submap_rg);///当前submap的点云，以当前关键帧代表
        ndt_matcher.setInputTarget(cloud_submap_rg);///目标subamp局部地图点云*/

        auto VoxelFilter = [=](const CloudPtr& input_cloud,CloudPtr& output_cloud, const float leaf_size){
            pcl::VoxelGrid<PointType> voxel_grid_filter;
            voxel_grid_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
            voxel_grid_filter.setInputCloud(input_cloud);
            voxel_grid_filter.filter(*output_cloud);
       };

        CloudPtr cloud_kf_submap_f(new PointCloudType);///关键帧的点云在其submap坐标系
        CloudPtr cloud_submap_f(new PointCloudType);///关键帧的点云在其submap坐标系
        VoxelFilter(cloud_submap, cloud_submap_f, 0.1);
        VoxelFilter(cloud_kf_submap, cloud_kf_submap_f, 0.1);
        ndt_matcher.setInputSource(cloud_kf_submap_f);///当前submap的点云，以当前关键帧代表
        ndt_matcher.setInputTarget(cloud_submap_f);///目标subamp局部地图点云
        PointCloudType unused_result;
        ndt_matcher.align(unused_result, input_guess.matrix().cast<float>());

        double trans_probability = ndt_matcher.getTransformationProbability();
//        if(trans_probability > 2.2 && ndt_matcher.getFitnessScore() < 0.83){
        if(trans_probability > 2.2){
            LOG(INFO)<<"trans_probability: "<<trans_probability<<", getFitnessScore: "<<ndt_matcher.getFitnessScore()
                     <<"历史submap id: "<<lc.smid<<" ， 当前帧submap ID： "<<keyframs_by_id[lc.kfid]->kf_in_submap_id_;
            M4d m = ndt_matcher.getFinalTransformation().cast<double>();
            Quat q(m.block<3, 3>(0, 0));
            q.normalize();
            SE3 pose = SE3(q, m.block<3, 1>(0, 3));

            mapping::common::LoopCandidate tmp_lc(lc.kfid, lc.smid);
            tmp_lc.relat_pose = pose;
            tmp_lc.smid_kf_in = keyframs_by_id[lc.kfid]->kf_in_submap_id_;
            impl_->final_loop_candidates_.emplace_back(tmp_lc);

            SE3 corect_pose = submaps_by_id[lc.smid]->GetLocalPose()* pose;
            impl_->ml_tracking_->UpdateSubMapPose(keyframs_by_id[lc.kfid]->kf_in_submap_id_,
                                                  corect_pose);
        }

        cloud_submap->clear(); cloud_submap = nullptr;
        cloud_kf_submap->clear(); cloud_kf_submap = nullptr;
    }

    for(auto &id:kf_id2clc_vector){
        keyframs_by_id[id]->cloud_ptr_->clear(); keyframs_by_id[id]->cloud_ptr_ = nullptr;
    }

    LOG(INFO)<<"检测成功 回环个数： "<<impl_->final_loop_candidates_.size();

    return;
    if(impl_->final_loop_candidates_.size()>0){
        PoseOptimization(impl_->final_loop_candidates_);
//        PoseOptimizationCeres(impl_->final_loop_candidates_);
        update_pose_ = true;
    }
}

void MLLoopClosing::DectectLoop(const std::shared_ptr<MLFrame> &cur_cf) {
    auto submaps_by_id = impl_->ml_tracking_->GetAllSubMap();///所有的submaps
    if(submaps_by_id.empty()){
        LOG(WARNING)<<"submaps_by_id.empty() WHEN ComputeOccupancyMap";
        return;
    }

    if(submaps_by_id.find(cur_cf->kf_in_submap_id_) == submaps_by_id.end())
        return;

    ///当前帧在世界坐标系下的位姿，通过其在submap中的坐标加上所在submap的位姿 得到
    SE3 submap_to_world = submaps_by_id[cur_cf->kf_in_submap_id_]->GetLocalPose();
    SE3 kf_w = submap_to_world * cur_cf->Tsl_;

    double range_th = 60.;
    for(auto &sm: submaps_by_id){
        if(submaps_by_id[cur_cf->kf_in_submap_id_]->id_ - sm.second->id_ < 1) continue;

        V3d dt = kf_w.translation() - sm.second->GetLocalPose().translation();///当前关键帧和历史中submap之间的距离
        double t2d = dt.head<2>().norm();  // x-y distance
        if(t2d<range_th){
//            LOG(INFO)<<"候选：submaps_by_id "<<submaps_by_id[cur_cf->kf_in_submap_id_]->id_<<"sm.second->id_: "<<sm.second->id_;
            std::unique_lock<std::mutex> lcm(impl_->loop_canditate_mutex_);
            impl_->loop_candidates_.emplace_back(
                    mapping::common::LoopCandidate(cur_cf->keyframe_id_ , sm.second->id_)
            );
            if(impl_->loop_candidates_.size()>4){
                impl_->loop_candidates_.pop_front();
            }
        }
    }

    if(impl_->loop_candidates_.size()>0){
        std::unique_lock<std::mutex>    lm(loopclosure_mutex_);
        loopclosure_cond_.notify_one();
    }
    submaps_by_id.clear();
}

void MLLoopClosing::PoseOptimization(std::deque<mapping::common::LoopCandidate>& cur_loop_candidates) {
    IdType start_submap_id = 99999999, end_submap_id= 1;

    auto closeloop_map_data = impl_->ml_tracking_->GetAllSubMap();///所有的submaps
    auto keyframs_by_id = impl_->ml_tracking_->GetKeyframByID();///所有的关键帧


    for(auto &candi: cur_loop_candidates){
//        LOG(INFO)<<"历史中的submap的id: "<<candi.smid<<" , 当前关键帧sumapID: "<<submaps_by_id[keyframs_by_id[candi.kfid]->kf_in_submap_id_]->id_;
        start_submap_id = std::min(start_submap_id,candi.smid);
        end_submap_id = std::max(end_submap_id,closeloop_map_data[keyframs_by_id[candi.kfid]->kf_in_submap_id_]->id_);
    }

    LOG(ERROR)<<"start_submap_id & end_submap_id: "<<start_submap_id<<" , "<<end_submap_id;


    auto *solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    /// 添加顶点
    AddVertex(optimizer, closeloop_map_data, start_submap_id , end_submap_id);

    int edge_id = 0;
    ///添加回环边
    AddLoopClosureFactors(optimizer, cur_loop_candidates, edge_id);
    /// 添加submap之间边
    AddSubmapConectFactors(optimizer, closeloop_map_data, start_submap_id , end_submap_id, edge_id);

    // 执行优化
    LOG(INFO) << "Optimization";
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(10);

//    optimizer.save("/home/idriver/catkin_ws/src/map_saved/trash_00/optimizer.g2o");

    /// 统计优化信息
    /// 根据第一次优化结果剔除loop的outlier edges

    for (auto v : optimizer.vertices()) {
        g2o::VertexSE3 * v_p
                = dynamic_cast< g2o::VertexSE3 * > (v.second);
        impl_->ml_tracking_->UpdateSubMapPose(v_p->id(), SE3(v_p->estimate().matrix()));
    }
//    delete solver;
}

///添加顶点， submap 的位姿
void MLLoopClosing::AddVertex(g2o::SparseOptimizer& optimizer,
                              const std::map<Idtype, std::shared_ptr<MLSubmap>>& closeloop_map_data,
                              IdType start_submap_id , IdType end_submap_id) {

    g2o::ParameterSE3Offset* odomOffset=new g2o::ParameterSE3Offset();
    odomOffset->setId(0);
    optimizer.addParameter(odomOffset);

    for(int id = start_submap_id;id<=end_submap_id;id++){
        if(closeloop_map_data.find(id) != closeloop_map_data.end())
        {
            std::shared_ptr<MLSubmap> submap = closeloop_map_data.find(id)->second;
            auto *v = new g2o::VertexSE3();
            SE3 pose = submap->GetLocalPose();
            v->setEstimate(SE3ToG2OSE3Quat(pose));
            v->setId(id);
            if(id == start_submap_id){
                v->setFixed(true);
            }
            optimizer.addVertex(v);

            g2o::EdgeSE3Prior* planeConstraint=new g2o::EdgeSE3Prior();
            M6d pinfo = M6d::Zero();
            pinfo(0, 0) = 40;
            pinfo(1, 1) = 40;
            pinfo(2, 2) = 40;
            pinfo(3, 3) = 40;
            pinfo(4, 4) = 40;
            pinfo(5, 5) = 40;
            planeConstraint->setInformation(pinfo);
            planeConstraint->setMeasurement(SE3ToG2OSE3Quat(pose));
//            planeConstraint->vertices()[0]=v;
            planeConstraint->setVertex(0, optimizer.vertex(id));
            planeConstraint->setParameterId(0, 0);
            optimizer.addEdge(planeConstraint);
//                impl_->vertices_.emplace_back(v);
        }else{
            LOG(ERROR)<<"回环检测到的submap不存在，这不可能";
        }
    }
}

void MLLoopClosing::AddLoopClosureFactors(g2o::SparseOptimizer& optimizer,
                                          std::deque<mapping::common::LoopCandidate>& cur_loop_candidates,
                                          int &edge_id) {
    const double loop_noise_pos = 0.2;
    const double loop_noise_ang = 2.5 / 180.0 * M_PI;
//    const double loop_th = 2.8; ///        const double loop_th = impl_->optimization_params_.loop_chi2_th;loop_chi2_th: 2.8
    const double loop_th = 4.5; ///        const double loop_th = impl_->optimization_params_.loop_chi2_th;loop_chi2_th: 2.8
    const double loop_weight = 50.0 ; ///impl_->optimization_params_.loop_weight, loop_weight: 5.0

    for (auto &lc : cur_loop_candidates) {
        auto *e = new g2o::EdgeSE3();
        e->setId(edge_id++);
        e->setVertex(0, optimizer.vertex(lc.smid));
        e->setVertex(1, optimizer.vertex(lc.smid_kf_in));
        e->setMeasurement(SE3ToG2OSE3Quat(lc.relat_pose));
        M6d info = M6d::Zero();
        info(0, 0) = 60;
        info(1, 1) = 60;
        info(2, 2) = 60;
        info(3, 3) = 60;
        info(4, 4) = 60;
        info(5, 5) = 60;
        e->setInformation(info);

        auto *rk = new g2o::RobustKernelCauchy;
        rk->setDelta(loop_th);
        e->setRobustKernel(rk);

        optimizer.addEdge(e);
//            impl_->loop_edges_.push_back(e);
    }
}

void MLLoopClosing::AddSubmapConectFactors(g2o::SparseOptimizer& optimizer,
                                           const std::map<Idtype, std::shared_ptr<MLSubmap>>& closeloop_map_data,
                                           IdType start_submap_id , IdType end_submap_id,
                                           int &edge_id){
    int lidar_continous_num = 2;
    for(int id = start_submap_id;id<end_submap_id;id++){
        for(int add_num = 1;add_num<lidar_continous_num; add_num++){
            if(closeloop_map_data.find(id) != closeloop_map_data.end() && id + add_num <= end_submap_id &&
               closeloop_map_data.find(id+add_num) != closeloop_map_data.end()
               ){
                std::shared_ptr<MLSubmap> submap_first = closeloop_map_data.find(id)->second;
                std::shared_ptr<MLSubmap> submap_second = closeloop_map_data.find(id+add_num)->second;


                SE3 obs_matching = submap_first->GetLocalPose().inverse() * submap_second->GetLocalPose();
//                    V6d noise_matching = submap_second->matching_noise_;

                auto *e = new g2o::EdgeSE3();
                e->setId(edge_id++);
                e->setVertex(0, optimizer.vertex(submap_first->id_));
                e->setVertex(1, optimizer.vertex(submap_second->id_));
                e->setMeasurement(SE3ToG2OSE3Quat(obs_matching));

                M6d info = M6d::Zero();
                info(0, 0) = 20;
                info(1, 1) = 20;
                info(2, 2) = 20;
                info(3, 3) = 20;
                info(4, 4) = 20;
                info(5, 5) = 20;
                e->setInformation(info);

//                    if (impl_->optimization_params_.use_rk_matching) {
//                        auto *rk = new g2o::RobustKernelCauchy;
//                        rk->setDelta(matching_th);
//                        e->setRobustKernel(rk);
//                    }

                optimizer.addEdge(e);
//                    impl_->matching_edges_.push_back(e);

            } else{
                LOG(ERROR)<<"回环检测到的submap不存在，这不可能";
            }
        }

    }
};

} }
