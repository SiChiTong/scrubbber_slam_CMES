//
// Created by gaoxiang on 2020/9/22.
//

#include "core/mt_search/multi_thread_search.h"
#include "common/mapping_math.h"
#include "common/std_headers.h"
#include "core/mem_control/mem_control.h"
#include "core/ndt_omp/include/pclomp/ndt_omp.h"
#include "core/resolution_matching/matcher/multi_resolution_matcher.h"
#include "core/resolution_matching/matcher/voxel_filter.h"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

namespace mapping::core {

using common::LoopCandidate;

MultiThreadSearch::MultiThreadSearch(const std::string &local_db_path, MTSearchParams params,
                                     std::map<IdType, common::KFPtr> &keyframes_by_id, bool run_as_gem)
    : param_(params),
      mem_control_(new core::MemoryControl(local_db_path, 100, run_as_gem)),
      keyframes_by_id_(keyframes_by_id) {
    if (param_.num_threads <= 0) {
        param_.num_threads = std::thread::hardware_concurrency();
        LOG(INFO) << "setting threads to " << param_.num_threads;
    }
}

int MultiThreadSearch::ComputeConstraint(const std::vector<LoopCandidate> &candidates) {
    candidate_results_.clear();
    workers_.clear();
    // create workers
    for (int i = 0; i < param_.num_threads; ++i) {
        std::shared_ptr<MTSearchWorker> w(new MTSearchWorker(i, keyframes_by_id_, range_, param_, mem_control_));
        w->SetFinishCallBack([this](const LoopCandidate &c) {
            UL lock(this->result_mutex_);
            candidate_results_.push_back(c);
        });

        workers_.insert({i, w});
    }

    for (int i = 0; i < candidates.size(); ++i) {
        int worker_index = i % param_.num_threads;
        workers_[worker_index]->AssignCandidateToSearch(candidates[i]);
    }

    LOG(INFO) << "Start multi thread matching, workers: " << workers_.size() << ", threads: " << param_.num_threads
              << ", candi: " << candidates.size();

    StartMatching();
    return 0;
}

void MultiThreadSearch::StartMatching() {
    for (auto &wp : workers_) {
        wp.second->Init();
        wp.second->Start();
    }

    for (auto &wp : workers_) {
        wp.second->Join();
    }

    // save results and filter the bad ones
    std::vector<LoopCandidate> accepted_results;
    for (auto &c : candidate_results_) {
        if (param_.registration_method == "ICP" || param_.registration_method == "GICP") {
            if (c.score < param_.fitness_score_thresh) {
                accepted_results.emplace_back(c);
                continue;
            }
        }
        if (c.score > param_.ndt_matching_min_proba) {
            accepted_results.emplace_back(c);
        }
    }
    LOG(INFO) << "accepted/total: " << accepted_results.size() << "/" << candidate_results_.size();

    candidate_results_ = std::move(accepted_results);
    LOG(INFO) << "mt search done.";
}

MTSearchWorker::MTSearchWorker(int index, std::map<IdType, common::KFPtr> &keyframes,
                               core::MultiResolutionMap::MapRange &range, MTSearchParams &param,
                               std::shared_ptr<core::MemoryControl> mem_control)
    : id_(index),
      keyframes_(keyframes),
      range_(range),
      param_(param),
      mem_control_(std::move(mem_control)),
      finished_(false) {
    Reset();
}

void MTSearchWorker::Reset() {
    float cell = range_.low_resolution * pow(2, range_.depth_resolution - 1);
    float step_deg = core::PointsTool::ComputeAngularSearchStep(cell, param_.lidar_max_range);
    int linear_search_window = int(param_.linear_search_meter / cell + 0.7);
    int angular_search_window = int(param_.angular_search_deg * M_PI / 180 / step_deg + 0.7);

    multi_matcher_ = std::make_shared<core::MultiResolutionMatcher>(range_);
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
    mem_control_ = nullptr;
    multi_matcher_ = nullptr;
}

void MTSearchWorker::AssignCandidateToSearch(common::LoopCandidate lc) {
    candidates_to_search_.emplace_back(std::move(lc));
}

void MTSearchWorker::Run() {
    for (auto &lc : candidates_to_search_) {
        current_candidate_ = lc;
        kf_a_ = keyframes_[lc.kfid_first];
        kf_b_ = keyframes_[lc.kfid_second];

        if (param_.registration_method == "ICP" || param_.registration_method == "GICP") {
            MatchWithICP();
        } else if (lc.use_mm_match) {
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
    auto map = CreateNodeSubmap(kf_a_, 40);

    if (map == nullptr || map->points.empty()) {
        return;
    }

    auto cloud_b = mem_control_->RequestCloud(kf_b_);
    if (cloud_b == nullptr || cloud_b->points.empty()) {
        return;
    }

    SE3 pose_a, pose_b;
    if (current_candidate_.use_pose_stage == 1) {
        pose_a = kf_a_->optimized_pose_stage_1_;
        if (current_candidate_.use_init_guess) {
            pose_b = pose_a * current_candidate_.Tij;
        } else {
            pose_b = kf_b_->optimized_pose_stage_1_;
        }
    } else if (current_candidate_.use_pose_stage == 2) {
        pose_a = kf_a_->optimized_pose_stage_2_;
        if (current_candidate_.use_init_guess) {
            pose_b = pose_a * current_candidate_.Tij;
        } else {
            pose_b = kf_b_->optimized_pose_stage_2_;
        }
    }

    ResetMatcherPtr(pose_a.translation()[0], pose_a.translation()[1]);
    multi_matcher_->AddPoints(map, param_.ground_height + pose_a.translation()[2]);

    auto npb_test = core::PointsTool::RemoveGround(cloud_b, 1);

    if (npb_test->size() < 1000) {
        finished_ = true;
        return;
    }

    auto npb = core::PointsTool::RemoveGround(cloud_b, 0.1);
    auto posebg = PoseToGround(pose_a, pose_b);

    SE3 pose_b_estimate;
    current_candidate_.score = multi_matcher_->Score(npb, param_.ground_height + pose_a.translation()[2], posebg,
                                                     param_.multi_matching_min_score, pose_b_estimate);

    auto end_time = std::chrono::steady_clock::now();
    auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    time_used_mm_match_ += time_used.count();

    if (current_candidate_.score > 0) {
        start_time = std::chrono::steady_clock::now();
        float trans_prob = NDT3D(map, npb, pose_b_estimate, false);

        end_time = std::chrono::steady_clock::now();
        auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
        time_used_ndt_ += time_used.count();

        current_candidate_.Tij = pose_a.inverse() * pose_b_estimate;
        current_candidate_.score = trans_prob;

        // call back
        finish_cb_(current_candidate_);
    }
}

void MTSearchWorker::MatchWithNDT() {
    /// 对于贴边的回环，永远都是第一个为贴边关键帧，第二个是建图关键帧
    auto map = CreateNodeSubmap(kf_b_, 40);
    if (map->points.empty()) {
        return;
    }

    SE3 pose_a;
    SE3 pose_b;

    if (current_candidate_.use_pose_stage == 1) {
        pose_b = kf_b_->optimized_pose_stage_1_;
    } else if (current_candidate_.use_pose_stage == 2) {
        pose_b = kf_b_->optimized_pose_stage_2_;
    }

    if (current_candidate_.use_init_guess) {
        pose_a = pose_b * current_candidate_.Tij.inverse();
    } else if (current_candidate_.use_pose_stage == 1) {
        pose_a = kf_a_->optimized_pose_stage_1_;
    } else if (current_candidate_.use_pose_stage == 2) {
        pose_a = kf_a_->optimized_pose_stage_2_;
    }

    auto cloud_a = mem_control_->RequestCloud(kf_a_);
    if (cloud_a == nullptr || cloud_a->points.empty()) {
        return;
    }

    auto npa = core::PointsTool::RemoveGround(cloud_a, 0.1);
    float trans_prob = -1.0;
    if (ndt_map_remove_ground_) {
        auto npb = core::PointsTool::RemoveGround(map, 0.1);
        trans_prob = NDT3D(npb, npa, pose_a, true);
    }
    else {
        trans_prob = NDT3D(map, npa, pose_a, false);
    }

    if (trans_prob <= param_.ndt_matching_min_proba) {
        return;
    }

    LoopCandidate candidate(kf_a_->id_, kf_b_->id_);
    candidate.Tij = pose_a.inverse() * pose_b;
    candidate.score = trans_prob;
    // call back
    finish_cb_(candidate);
}

void MTSearchWorker::MatchWithICP() {
    auto map = CreateNodeSubmap(kf_b_, 1);
    if (map->points.empty()) {
        return;
    }

    SE3 pose_a;
    SE3 pose_b;

    if (current_candidate_.use_pose_stage == 1) {
        pose_b = kf_b_->optimized_pose_stage_1_;
    } else if (current_candidate_.use_pose_stage == 2) {
        pose_b = kf_b_->optimized_pose_stage_2_;
    }

    if (current_candidate_.use_init_guess) {
        pose_a = pose_b * current_candidate_.Tij.inverse();
    } else if (current_candidate_.use_pose_stage == 1) {
        pose_a = kf_a_->optimized_pose_stage_1_;
    } else if (current_candidate_.use_pose_stage == 2) {
        pose_a = kf_a_->optimized_pose_stage_2_;
    }

    auto cloud_a = mem_control_->RequestCloud(kf_a_);
    if (cloud_a == nullptr || cloud_a->points.empty()) {
        LOG(INFO) << "Point cloud is empty";
        return;
    }

    auto npa = core::PointsTool::RemoveGround(cloud_a, 0.1);
    auto npb = core::PointsTool::RemoveGround(map, 0.1);

    M4d relative = pose_a.matrix().inverse() * pose_b.matrix();

    double trans_prob = ICP(npb, npa, static_cast<M4d &>(relative), false);

    if (trans_prob >= param_.fitness_score_thresh) {
        return;
    }

    LoopCandidate candidate(kf_a_->id_, kf_b_->id_);
    candidate.Tij = pose_a.inverse() * pose_b;
    candidate.score = trans_prob;
    // call back
    finish_cb_(candidate);
}

common::PointCloudType::Ptr MTSearchWorker::CreateNodeSubmap(const common::KFPtr &node, int num) {
    common::PointCloudType::Ptr map(new common::PointCloudType());
    common::PointCloudType::Ptr tem(new common::PointCloudType());

    const int step = 3;
    int cnt_kfs = 0;
    for (int i = -num; i <= num; i += step) {
        auto ite = keyframes_.find(node->id_ + i);
        if (ite != keyframes_.end() && ite->second->trajectory_id_ == node->trajectory_id_) {
            auto cloud = mem_control_->RequestCloud(ite->second);
            if (cloud == nullptr || cloud->points.empty()) {
                continue;
            }

            if (current_candidate_.use_pose_stage == 1) {
                pcl::transformPointCloud(*cloud, *tem, ite->second->optimized_pose_stage_1_.cast<float>().matrix());
            } else {
                pcl::transformPointCloud(*cloud, *tem, ite->second->optimized_pose_stage_2_.cast<float>().matrix());
            }

            (*map) += (*tem);
            cnt_kfs++;
        }
    }

    // LOG(INFO) << "submap for " << node->id_ << " contains kfs: " << cnt_kfs;

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

float MTSearchWorker::NDT3D(common::PointCloudType::Ptr map, common::PointCloudType::Ptr points, SE3 &pose,
                            bool save_result) {
    SE3 input_guess = pose;
    common::PointCloudType::Ptr map_v(new common::PointCloudType());
    common::PointCloudType::Ptr ps_v(new common::PointCloudType());
    pcl::VoxelGrid<common::PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
    voxel_grid_filter.setInputCloud(map);
    voxel_grid_filter.filter(*map_v);
    voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
    voxel_grid_filter.setInputCloud(points);
    voxel_grid_filter.filter(*ps_v);

    pcl::NormalDistributionsTransform<common::PointType, common::PointType> ndt_matcher;
    ndt_matcher.setResolution(1);
    ndt_matcher.setStepSize(0.1);
    ndt_matcher.setTransformationEpsilon(param_.transformation_epsilon);
    ndt_matcher.setMaximumIterations(param_.max_iterations);
    ndt_matcher.setInputSource(ps_v);
    ndt_matcher.setInputTarget(map_v);
    common::PointCloudType unused_result;
    ndt_matcher.align(unused_result, input_guess.matrix().cast<float>());

    double trans_probability = ndt_matcher.getTransformationProbability();
    M4d m = ndt_matcher.getFinalTransformation().cast<double>();
    Quat q(m.block<3, 3>(0, 0));
    q.normalize();
    pose = SE3(q, m.block<3, 1>(0, 3));

    if (save_result && trans_probability > param_.ndt_matching_min_proba) {
        common::PointCloudType merged;
        merged += *map;
        merged += unused_result;
        pcl::io::savePCDFileBinaryCompressed(std::to_string(kf_a_->id_) + "_" + std::to_string(kf_b_->id_) + "_" +
                                                 std::to_string(trans_probability) + ".pcd",
                                             merged);
    }

    // if (ndt_matcher.lambda5() < 10000) {
    //     return 0;
    // }

    return trans_probability;
}

double MTSearchWorker::ICP(common::PointCloudType::Ptr map, common::PointCloudType::Ptr points, M4d &pose,
                           bool save_result) {
    auto input_guess = pose;

    common::PointCloudType::Ptr map_v(new common::PointCloudType());
    common::PointCloudType::Ptr ps_v(new common::PointCloudType());

    pcl::VoxelGrid<common::PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
    voxel_grid_filter.setInputCloud(map);
    voxel_grid_filter.filter(*map_v);
    voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
    voxel_grid_filter.setInputCloud(points);
    voxel_grid_filter.filter(*ps_v);

    pcl::Registration<common::PointType, common::PointType>::Ptr registration;
    if (param_.registration_method == "ICP") {
        auto icp = boost::make_shared<pcl::IterativeClosestPoint<common::PointType, common::PointType>>();
        registration = icp;
    } else if (param_.registration_method == "GICP") {
        auto gicp = boost::make_shared<pcl::GeneralizedIterativeClosestPoint<common::PointType, common::PointType>>();
        registration = gicp;
    }
    registration->setTransformationEpsilon(param_.transformation_epsilon);
    registration->setMaximumIterations(param_.max_iterations);
    registration->setInputSource(ps_v);
    registration->setInputTarget(map_v);
    common::PointCloudType unused_result;
    registration->align(unused_result, input_guess.cast<float>());

    Eigen::Isometry3d relative;
    relative.matrix() = registration->getFinalTransformation().cast<double>();
    double fitness_score = CalcFitnessScore(ps_v, map_v, relative, param_.fitness_score_max_range);
    if (fitness_score < INT8_MAX) {
        LOG(INFO) << "fitness_score: " << fitness_score;
    }
    if (save_result && fitness_score < param_.fitness_score_thresh) {
        common::PointCloudType merged;
        merged += *map;
        merged += unused_result;
        pcl::io::savePCDFileBinaryCompressed(std::to_string(kf_a_->id_) + "_" + std::to_string(kf_b_->id_) + "_" +
                                                 std::to_string(fitness_score) + ".pcd",
                                             merged);
    }

    return fitness_score;
}

SE3 MTSearchWorker::PoseToGround(const SE3 &posea, const SE3 &poseb) {
    auto pose_a = common::SE3ToRollPitchYaw(posea);
    auto pose_b = common::SE3ToRollPitchYaw(poseb);

    double dyaw = fabs(pose_a.yaw - pose_b.yaw);
    if (dyaw < 2 * M_PI && dyaw > M_PI) {
        pose_b.roll = -pose_a.roll;
        pose_b.pitch = -pose_a.pitch;
    } else {
        pose_b.roll = pose_a.roll;
        pose_b.pitch = pose_a.pitch;
    }

    pose_b.z = pose_a.z;
    return common::XYZRPYToSE3(pose_b);
}

double MTSearchWorker::CalcFitnessScore(const common::PointCloudType::ConstPtr &cloud1,
                                        const common::PointCloudType::ConstPtr &cloud2,
                                        const Eigen::Isometry3d &relpose, double max_range) {
    pcl::search::KdTree<common::PointType>::Ptr tree_(new pcl::search::KdTree<common::PointType>());
    tree_->setInputCloud(cloud1);

    double fitness_score = 0.0;

    // Transform the input dataset using the final transformation
    pcl::PointCloud<common::PointType> input_transformed;
    pcl::transformPointCloud(*cloud2, input_transformed, relpose.cast<float>());

    std::vector<int> nn_indices(1);
    std::vector<float> nn_dists(1);

    // For each point in the source dataset
    int nr = 0;
    for (size_t i = 0; i < input_transformed.points.size(); ++i) {
        // Find its nearest neighbor in the target
        tree_->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);

        // Deal with occlusions (incomplete targets)
        if (nn_dists[0] <= max_range) {
            // Add to the fitness score
            fitness_score += nn_dists[0];
            nr++;
        }
    }

    if (nr > 0) return (fitness_score / nr);
    else
        return (std::numeric_limits<double>::max());
}

}  // namespace mapping::core