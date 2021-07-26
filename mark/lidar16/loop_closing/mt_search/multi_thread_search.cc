//
// Created by herenjie on 2020/11/10.
//

#include <pcl/filters/voxel_grid.h>
#include <lidar16/loop_closing/resolution_matching/matcher/voxel_filter.h>
#include <pcl/io/pcd_io.h>
#include "multi_thread_search.h"
namespace scrubber_slam { namespace lidar16{
    MultiThreadSearch::MultiThreadSearch(MTSearchParams params,
                                         std::shared_ptr<MLTracking> ml_tracker)
            : param_(params){
//        keyframes_by_id_ = ml_tracker->impl_->keyframes_by_id_;
//        closeloop_map_data_ = ml_tracker->impl_->closeloop_map_data_;
        ml_tracker_ = ml_tracker;

        if (param_.num_threads <= 0) {
            param_.num_threads = std::thread::hardware_concurrency();
            LOG(INFO) << "setting threads to " << param_.num_threads;
        }
    }

    int MultiThreadSearch::ComputeConstraint(const std::vector<mapping::common::LoopCandidate> &candidates) {
        candidate_results_.clear();
        workers_.clear();
        // create workers,每一个线程一个worker
        std::map<IdType, std::shared_ptr<MLFrame>> keyframes_by_id = ml_tracker_->GetKeyframByID();
        std::map<Idtype, std::shared_ptr<MLSubmap>> closeloop_map_data = ml_tracker_->GetAllSubMap();//回环检测用的submap的ID和submap
        for (int i = 0; i < param_.num_threads; ++i) {
            std::shared_ptr<MTSearchWorker> w(new MTSearchWorker(i, keyframes_by_id,closeloop_map_data, param_));
            w->SetFinishCallBack([this](const mapping::common::LoopCandidate &c) {
                UL lock(this->result_mutex_);
                candidate_results_.push_back(c);
            });

            workers_.insert({i, w});
        }

        ///把回环候选分配到对应的worker核心上
        for (int i = 0; i < candidates.size(); ++i) {
            int worker_index = i % param_.num_threads;
            workers_[worker_index]->AssignCandidateToSearch(candidates[i]);
        }

        StartMatching();
        return 0;
    }

    void MultiThreadSearch::StartMatching() {
        LOG(INFO) << "Start multi thread matching, workers: " << workers_.size() << ", threads: " << param_.num_threads;
        ///启动每一个worker
        for (auto &wp : workers_) {
            wp.second->Start();
        }
        ///等待每一个worker结束
        for (auto &wp : workers_) {
            wp.second->Join();
        }

        // save results and filter the bad ones
        std::vector<mapping::common::LoopCandidate> accepted_results;
        for (auto &c : candidate_results_) {
            LOG(ERROR)<<"c.score: "<<c.score<<"param_.ndt_matching_min_proba: "<<param_.ndt_matching_min_proba;
            if (c.score > param_.ndt_matching_min_proba) {
                accepted_results.emplace_back(c);
            }
        }

        candidate_results_ = std::move(accepted_results);
        LOG(INFO) << "mt search done.";
    }



////////////////MTSearchWorker///////////////////////////////////////
    MTSearchWorker::MTSearchWorker(int index, std::map<IdType, std::shared_ptr<MLFrame>> &keyframes,
                                   std::map<Idtype, std::shared_ptr<MLSubmap>> closeloop_map_data,
                                   MTSearchParams &param)
            : id_(index),//线程编号
              keyframes_(keyframes),//id, 关键帧
              closeloop_map_data_(closeloop_map_data),//id, submap
              param_(param),
              finished_(false) {
        Reset();
    }

    void MTSearchWorker::Reset() {
    float cell = range_.low_resolution * pow(2, range_.depth_resolution - 1);
    float step_deg = PointsTool::ComputeAngularSearchStep(cell, param_.lidar_max_range);
    int linear_search_window = int(param_.linear_search_meter / cell + 0.7);
    int angular_search_window = int(param_.angular_search_deg * M_PI / 180 / step_deg + 0.7);

    multi_matcher_ = std::make_shared<MultiResolutionMatcher>(range_);
    multi_matcher_->SetSearchWindow(linear_search_window, angular_search_window);
    multi_matcher_->SetMaxRange(param_.lidar_max_range);
}

    void MTSearchWorker::Start() {
        job_ = std::thread([this]() { this->Run(); });
    }

    void MTSearchWorker::Join() {
        job_.join();
        // clean data
        kf_a_ = nullptr;
        kf_b_ = nullptr;
    multi_matcher_ = nullptr;
    }

void MTSearchWorker::AssignCandidateToSearch(mapping::common::LoopCandidate lc) {
    candidates_to_search_.emplace_back(lc);
}

void MTSearchWorker::Run() {
    for (auto &lc : candidates_to_search_) {
        current_candidate_ = lc;
        kf_a_ = keyframes_[lc.kfid_first];///当前帧要用来检测会换的关键帧
        kf_b_ = keyframes_[lc.kfid_second];///历史帧中用于和当前帧进行匹配，判断是否构成回环的帧

        if (lc.use_mm_match){
            MatchWithMM();
        } else {
            MatchWithNDT();
        }
    }

    finished_ = true;
    // LOG(INFO) << "average time usage for mm match: " << time_used_mm_match_ / candidates_to_search_.size()
    //           << ", for ndt: " << time_used_ndt_ / candidates_to_search_.size();
}

void MTSearchWorker::MatchWithMM() {
    Reset();
    auto start_time = std::chrono::steady_clock::now();

    /// 用node附近的且和kf_b_在同一个submap中的关键帧创建局部点云, world系,
    auto map = CreateNodeSubmap(kf_b_, 100);
    ///下面代码是把关键帧所在的submap的所有关键帧放在一起
    Idtype kf_b_loop_submap_id = kf_b_->loop_submap_id_;///历史关键帧在哪一个回环检测的submap中;
    std::shared_ptr<MLSubmap> kf_b_candidate_submap = closeloop_map_data_[kf_b_loop_submap_id];

    SE3 pose_a, pose_b;

    ///历史关键帧世界坐标
    Idtype loop_submap_id = kf_b_->loop_submap_id_;///历史关键帧在哪一个回环检测的submap中;
    std::shared_ptr<MLSubmap> kf_candidate_submap = closeloop_map_data_[loop_submap_id];
    pose_b =  kf_candidate_submap->local_pose_ * kf_b_->Tsi_ ;

//    pose_b = kf_b_->Twi_;///历史关键帧世界坐标
///当前关键帧转到了submap，所以变为submap的世界坐标
    Idtype kf_a_loop_submap_id = kf_a_->loop_submap_id_;///关键帧在哪一个回环检测的submap中;
    std::shared_ptr<MLSubmap> candidate_submap = closeloop_map_data_[kf_a_loop_submap_id];
    pose_a = candidate_submap->GetLocalPose();///当前关键帧转到了submap，所以变为submap的世界坐标

    ResetMatcherPtr(pose_b.translation()[0], pose_b.translation()[1]);
    multi_matcher_->AddPoints(map, param_.ground_height + pose_b.translation()[2]);

    ///当前点云转到局部submap系下
    CloudPtr npa_submap(new PointCloudType);
    pcl::transformPointCloud(*kf_a_->cloud_ptr_, *npa_submap, kf_a_->Tsi_.matrix());

    auto npb_test = PointsTool::RemoveGround(npa_submap, 1);

    if (npb_test->size() < 1000) {
        finished_ = true;
        return;
    }

    auto npb = PointsTool::RemoveGround(npa_submap, 0.1);
    auto posebg = PoseToGround(pose_b, pose_a);///posea 平移 01 poseb 平移 2； YPR：  posea 0， poseb 12

    SE3 pose_a_estimate;
    current_candidate_.score = multi_matcher_->Score(npb, param_.ground_height + pose_b.translation()[2], posebg,
                                                     param_.multi_matching_min_score, pose_a_estimate);

    auto end_time = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    time_used_mm_match_ += time_used.count();

    if (current_candidate_.score > 0)
    {
        start_time = std::chrono::steady_clock::now();

        float trans_prob = NDT3D(map , npb, pose_a_estimate, false);

        end_time = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
        time_used_ndt_ += time_used.count();

        current_candidate_.Tij = pose_b.inverse() * pose_a_estimate;
        current_candidate_.Tij_submap = kf_b_candidate_submap->GetLocalPose().inverse() * pose_a_estimate;
        current_candidate_.smid_first = kf_a_loop_submap_id;///第一个,当前帧的所在的submap的ID
        current_candidate_.smid_second = kf_b_loop_submap_id;///历史关键帧在哪一个回环检测的submap中;
        current_candidate_.score = trans_prob;

        // call back
        finish_cb_(current_candidate_);

        CloudPtr cur_cloud_trans(new PointCloudType);
        pcl::transformPointCloud(*npb, *cur_cloud_trans, pose_a_estimate.matrix());


        if(trans_prob > param_.ndt_matching_min_proba)
        {
            int kf_id_0 = current_candidate_.kfid_first;
            int kf_id_1 = current_candidate_.kfid_second;
        }
    }
}

    void MTSearchWorker::MatchWithNDT() {
        Idtype loop_submap_id = kf_b_->loop_submap_id_;///历史关键帧在哪一个回环检测的submap中;

        std::shared_ptr<MLSubmap> candidate_submap = closeloop_map_data_[loop_submap_id];

        std::vector<std::shared_ptr<MLFrame>> keyframes_cur_submap = candidate_submap->keyframes_cur_submap_;///当前submap中存在所有的关键帧

        ///历史帧所在的loop submap中所有的关键帧的点云，用于和当前关键帧匹配，判断是否构成回环
        //TODO: 待确定把点云放在哪一个坐标系下
        CloudPtr submap_cloud(new PointCloudType);

        for(auto &kf: keyframes_cur_submap){
            ///当前激光点云转到全局世界坐标系下
            CloudPtr cur_cloud_trans(new PointCloudType);
            pcl::transformPointCloud(*kf->cloud_ptr_, *cur_cloud_trans, kf->Twi_.matrix());

            *submap_cloud += *cur_cloud_trans;
        }

        CloudPtr npa(new PointCloudType);///当前帧点云
        *npa = *kf_a_->cloud_ptr_;
        SE3 pose_a = kf_a_->Twi_;
        SE3 pose_b;

        float step = 15.0;
        float trans_prob = 999.;
        for(int i = -1;i<1; i++){
            Vec3 position(pose_a.translation()[0], pose_a.translation()[1], 0.0);
            Eigen::Matrix3d rostation_matrix= pose_a.rotationMatrix();
            Eigen::Vector3d euler_angles = rostation_matrix.eulerAngles(0, 1, 2);
            double yaw_rad = euler_angles[2] + step*float(i);
            Vec3 altitude(0.0, 0.0, yaw_rad);
            SE3 predict_pose1(SO3::exp(altitude), position);

            float tmp_trans_prob = NDT3D(submap_cloud, npa, pose_a, false);
            if(tmp_trans_prob<trans_prob){
                trans_prob = tmp_trans_prob;
            }
        }

//        LOG(ERROR)<<"CUR trans_prob : "<<trans_prob;
        if (trans_prob <= param_.ndt_matching_min_proba) {
            LOG(ERROR)<<"trans_prob <= param_.ndt_matching_min_proba : "<<trans_prob<<" / "<<param_.ndt_matching_min_proba;
            return;
        }

        mapping::common::LoopCandidate candidate(kf_a_->id_, kf_b_->id_);
        candidate.Tij = pose_a.inverse() * pose_b;
        candidate.score = trans_prob;
        // call back
        finish_cb_(candidate);
    }

    CloudPtr MTSearchWorker::CreateNodeSubmap(const std::shared_ptr<MLFrame> &keyframe, int num) {
        CloudPtr map(new PointCloudType());
        CloudPtr tem(new PointCloudType());

/*        (*map) += *keyframe->cloud_ptr_;
        const int step = 1;
        for (int i = -num; i <= num; i += step) {
            auto ite = keyframes_.find(keyframe->id_ + i);
            if (ite != keyframes_.end()) {
                auto cloud = ite->second->cloud_ptr_;
                pcl::transformPointCloud(*cloud, *tem, ite->second->Twi_.matrix());

                (*map) += (*tem);
            }
        }*/


        Idtype loop_submap_id = keyframe->loop_submap_id_;///历史关键帧在哪一个回环检测的submap中;
        std::shared_ptr<MLSubmap> kf_candidate_submap = closeloop_map_data_[loop_submap_id];
////    auto map = kf_candidate_submap->cloud_;
//
//        ///关键帧本身加入到map中, 转到世界坐标系下
        SE3 T_kf2world =  kf_candidate_submap->local_pose_ * keyframe->Tsi_ ;
        pcl::transformPointCloud(*keyframe->cloud_ptr_, *tem, T_kf2world.matrix());
//        pcl::transformPointCloud(*keyframe->cloud_ptr_, *tem, keyframe->Twi_.matrix());
        (*map) += (*tem);

        const int step = 1;
        int add_num = 0;
        int left_or_right = 1;
        for( int i=1; i<2*num; ){
            auto ite = keyframes_.find(keyframe->id_ + i * left_or_right);
            left_or_right = -left_or_right;///换向
            if( left_or_right == 1 ) i+=step;
//            if (ite != keyframes_.end() && loop_submap_id == ite->second->loop_submap_id_) {
            if (ite != keyframes_.end() ) {
                auto cloud = ite->second->cloud_ptr_;
//                pcl::transformPointCloud(*cloud, *tem, ite->second->Twi_.matrix());

                Idtype loop_submap_id = ite->second->loop_submap_id_;///历史关键帧在哪一个回环检测的submap中;
                std::shared_ptr<MLSubmap> kf_candidate_submap = closeloop_map_data_[loop_submap_id];
                SE3 T_kf2world = kf_candidate_submap->local_pose_ * ite->second->Tsi_;
                pcl::transformPointCloud(*cloud, *tem, T_kf2world.matrix());

                (*map) += (*tem);
                if( add_num++ > num ) break;
            }
        }

        return map;
    }

void MTSearchWorker::ResetMatcherPtr(double x, double y) {
    auto range = range_;
    range.offset_x = x - 100;
    range.offset_y = y - 100;
    range.length = 200;
    range.width = 200;
    multi_matcher_->Init(range);
}

float MTSearchWorker::NDT3D(PointCloudType::Ptr map, PointCloudType::Ptr points, SE3 &pose,
                            bool save_result) {
    SE3 input_guess = pose;
    PointCloudType::Ptr map_v(new PointCloudType());
    PointCloudType::Ptr ps_v(new PointCloudType());
    pcl::VoxelGrid<PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(0.1, 0.1, 0.1);
    voxel_grid_filter.setInputCloud(map);
    voxel_grid_filter.filter(*map_v);
    voxel_grid_filter.setLeafSize(0.1, 0.1, 0.1);
    voxel_grid_filter.setInputCloud(points);
    voxel_grid_filter.filter(*ps_v);

    pclomp::NormalDistributionsTransform<PointType, PointType> ndt_matcher;
    ndt_matcher.setResolution(1);
    ndt_matcher.setStepSize(0.01);
    ndt_matcher.setTransformationEpsilon(0.01);
    ndt_matcher.setMaximumIterations(200);
    ndt_matcher.setInputSource(ps_v);
    ndt_matcher.setInputTarget(map_v);
    PointCloudType unused_result;
    ndt_matcher.align(unused_result, input_guess.matrix().cast<float>());

    double trans_probability = ndt_matcher.getTransformationProbability();
    M4d m = ndt_matcher.getFinalTransformation().cast<double>();
    Quat q(m.block<3, 3>(0, 0));
    q.normalize();
    pose = SE3(q, m.block<3, 1>(0, 3));


//    if (ndt_matcher.lambda5() < 10000) {
//        return 0;
//    }

    return trans_probability;
}

///poseb 平移 01 posea 平移 2； YPR：  poseb 0， posea 12
SE3 MTSearchWorker::PoseToGround(const SE3 &posea, const SE3 &poseb) {
    V3d ypr_a = mapping::common::SE3ToYawPitchRoll(posea);
    V3d ypr_b = mapping::common::SE3ToYawPitchRoll(poseb);
    ypr_b[2] = ypr_a[2];///Z轴
    ypr_b[1] = ypr_a[1];///Y轴
    V3d t = poseb.translation();
    t[2] = posea.translation()[2];
    return mapping::common::XYZYPRToSE3(t, ypr_b);
}
} }