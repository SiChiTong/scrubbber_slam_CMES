//
// Created by gaoxiang on 2020/9/16.
//

#include "pipeline/optimization_s2/optimization_s2.h"
#include "common/gps_status_def.h"
#include "io/db_io.h"
#include "io/file_io.h"
#include "pipeline/optimization_s2/optimization_s2_impl.h"

#include <glog/logging.h>
#include <numeric>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <boost/format.hpp>

using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;

namespace mapping::pipeline {

using namespace mapping::common;

/// SE3 转g2o的SE3Quat
inline g2o::SE3Quat SE3ToG2OSE3Quat(const SE3 &T) { return g2o::SE3Quat(T.so3().matrix(), T.translation()); }

OptimizationStage2::OptimizationStage2(const io::YAML_IO &yaml_file, RunMode run_mode)
    : yaml_file_(yaml_file), impl_(new OptimizationStage2Impl), PipelineContext(run_mode) {
    context_name_ = "OptimizationStage2";
}

OptimizationStage2::~OptimizationStage2() noexcept { LOG(INFO) << "Opt s2 deconstructed."; }

bool OptimizationStage2::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }
    impl_->origin_info_.map_origin_x = yaml_file_.GetValue<double>("map_origin_param", "map_origin_x");
    impl_->origin_info_.map_origin_y = yaml_file_.GetValue<double>("map_origin_param", "map_origin_y");
    impl_->origin_info_.map_origin_z = yaml_file_.GetValue<double>("map_origin_param", "map_origin_z");
    impl_->origin_info_.map_origin_zone = yaml_file_.GetValue<int>("map_origin_param", "map_origin_zone");
    impl_->origin_info_.is_southern = yaml_file_.GetValue<bool>("map_origin_param", "is_southern");

    if (!get_keyframes_from_quick_debug_) {
        impl_->optimization_params_.LoadFromYAML(yaml_file_);
    }
    impl_->gps_status_ = common::BagGNSSStatusType(yaml_file_.GetValue<int>("gps_status"));
    impl_->gps_th_ = impl_->optimization_params_.gps_chi2_th;
    impl_->if_use_nav_sat_ = yaml_file_.GetValue<bool>("use_nav_sat");
    if_merge_maps_ = yaml_file_.GetValue<bool>("if_merge_maps");

    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");
    if (run_mode_ == PipelineContext::RunMode::GEM_EXECUTABLE) {
        local_db_path_ = yaml_file_.GetValue<std::string>("local_db_path");
    }

    std::string keyframes_txt_path;
    if (if_merge_maps_) {
        LogAndReport("加载新增地图合并的关键帧文件");
        keyframes_txt_path = local_data_path_ + "merge_keyframes.txt";
    } else {
        keyframes_txt_path = local_data_path_ + "keyframes.txt";
    }

    if (!get_keyframes_from_quick_debug_ && !io::LoadKeyframes(keyframes_txt_path, impl_->keyframes_)) {
        report_ += "Cannot load keyframes at " + local_data_path_ + "keyframes.txt\n";
        LOG(ERROR) << "Cannot load keyframes at " << local_data_path_ << "keyframes.txt\n";
        return false;
    }

    if (!io::LoadLoopCandidates(local_data_path_ + "loops.txt", impl_->loop_candidates_)) {
        report_ += "Cannot load keyframes at " + local_data_path_ + "keyframes.txt\n";
        LOG(ERROR) << "Cannot load keyframes at " << local_data_path_ << "keyframes.txt\n";
        return false;
    }
    if (if_merge_maps_) {
        if (!io::LoadMergeInfo(local_data_path_ + "merge_info.txt", impl_->merge_info_vec_)) {
            report_ += "Cannot load keyframes at " + local_data_path_ + "merge_info.txt\n";
            LOG(ERROR) << "Cannot load keyframes at " << local_data_path_ << "merge_info.txt\n";
            return false;
        }
    }

    if (get_keyframes_from_quick_debug_) {
        for (auto &kf : impl_->keyframes_map_by_id_) {
            impl_->keyframes_[kf.second->trajectory_id_].emplace_back(kf.second);
        }
    } else {
        for (auto &t : impl_->keyframes_) {
            for (const auto &kf : t.second) {
                if (impl_->keyframes_map_by_id_.find(kf->id_) == impl_->keyframes_map_by_id_.end()) {
                    impl_->keyframes_map_by_id_.insert({kf->id_, kf});
                }
            }
        }
    }

    LOG(INFO) << "trajectories: " << impl_->keyframes_.size();
    LOG(INFO) << "loop candidates: " << impl_->loop_candidates_.size();
    return true;
}

bool OptimizationStage2::Start() {
    LOG(INFO) << "Optimization stage 2";
    SetStatus(ContextStatus::WORKING);

    BuildProblem();

    /// 第一轮
    impl_->optimizer_.save((local_data_path_ + "s2_init.g2o").c_str());
    LogAndReport("求解第一次优化");
    Solve();

    /// 去outlier
    RemoveOutliers();

    AddHeightFactors();

    /// 第二轮
    LogAndReport("求解第二次优化");
    Solve();

    /// 打印信息
    CollectOptimizationStatistics();

    /// 存储第二轮优化结果
    LOG(INFO) << "saving g2o";
    impl_->optimizer_.save((local_data_path_ + "s2_final.g2o").c_str());

    double_t map_height = impl_->origin_info_.map_origin_z;

    //    if (UpdateMapHeight(map_height)) {
    //        LOG(INFO) << "Reset map height to: " << map_height;
    //        report_ += "pose optimization warning: map height from " +
    //        std::to_string(impl_->origin_info_.map_origin_z) +
    //                   " to " + std::to_string(map_height) + ".\n";
    //        impl_->origin_info_.map_origin_z = map_height;
    //        yaml_file_.SetValue<double>("map_origin_param", "map_origin_z", impl_->origin_info_.map_origin_z);
    //        yaml_file_.Save();
    //    }

    LOG(INFO) << "saving path";
    io::RemoveIfExist(local_data_path_ + "optimization2.txt");
    io::SaveKeyframeSinglePath(local_data_path_ + "optimization2.txt", impl_->keyframes_,
                               io::SaveKeyframePathType::OPTI_PATH_STAGE_2);

    LOG(INFO) << "saving gps path";
    io::RemoveIfExist(local_data_path_ + "gps_path.txt");
    io::SaveKeyframeSinglePath(local_data_path_ + "gps_path.txt", impl_->keyframes_,
                               io::SaveKeyframePathType::GPS_PATH);

    LOG(INFO) << "saving keyframes";
    std::string keyframes_txt_path;
    if (if_merge_maps_) {
        LogAndReport("加载新增地图合并的关键帧文件");
        keyframes_txt_path = local_data_path_ + "merge_keyframes.txt";
    } else {
        keyframes_txt_path = local_data_path_ + "keyframes.txt";
    }
    io::RemoveIfExist(keyframes_txt_path);
    io::SaveKeyframePose(keyframes_txt_path, impl_->keyframes_);

    if (save_pcd_) {
        SavePcd();
    }

    if (save_db_) {
        UpdateDb();
    }

    LOG(INFO) << "done.";
    SetStatus(ContextStatus::SUCCEED);

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        SaveGemReport(local_data_path_ + gem_report_name_);
    }

    return true;
}

void OptimizationStage2::BuildProblem() {
    auto *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    auto *offset_value = new g2o::ParameterSE3Offset();
    offset_value->setId(0);
    impl_->optimizer_.addParameter(offset_value);
    impl_->optimizer_.setAlgorithm(solver);

    // add vertex
    AddVertex();

    // 必需部分
    if (impl_->if_use_nav_sat_) {
        AddGpsFactors();
    }
    AddMatchingFactors();
    AddDRFactors();
    AddLoopClosureFactors();

    if (impl_->optimization_params_.with_global_rotation) {
        AddGlobalRotationFactors();
    }

    // 添加边：人工 FIX 约束
    AddFixFactors();
}

void OptimizationStage2::AddVertex() {
    auto find_trajectory_type = [this](const int &id) -> VertexOptimizationType {
        for (auto &mi : impl_->merge_info_vec_) {
            if (id == mi.trajectory_id_) {
                return mi.vertex_type_;
            }
        }
        return VertexOptimizationType::UNFIXED;
    };

    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
            if (kf->bag_type_ == KeyFrameBagType::VALIDATION_BAGS) {
                continue;
            }
            auto *v = new g2o::VertexSE3();
            // 地图新增需要添加固定点，保证主地图不受影响
            if (if_merge_maps_ && impl_->merge_info_vec_.size() > 0) {
                /// 主地图的固定点添加
                if (VertexOptimizationType::FIXED == find_trajectory_type(kf->trajectory_id_)) {
                    v->setEstimate(SE3ToG2OSE3Quat(kf->optimized_pose_stage_2_));
                    v->setId(kf->id_);
                    v->setFixed(true);
                } else {
                    v->setEstimate(SE3ToG2OSE3Quat(kf->optimized_pose_stage_1_));
                    v->setId(kf->id_);
                }
            } else {
                v->setEstimate(SE3ToG2OSE3Quat(kf->optimized_pose_stage_1_));
                v->setId(kf->id_);
                if (std::find(impl_->fixed_keyframe_indexes_.begin(), impl_->fixed_keyframe_indexes_.end(), kf->id_) !=
                    impl_->fixed_keyframe_indexes_.end()) {
                    v->setFixed(true);
                }
            }
            impl_->optimizer_.addVertex(v);
            impl_->vertices_.emplace_back(v);
        }
    }
}

void OptimizationStage2::AddGpsFactors() {
    double gps_th = impl_->optimization_params_.gps_chi2_th;
    for (auto &kfp : impl_->keyframes_map_by_id_) {
        if (std::find(impl_->fixed_keyframe_indexes_.begin(), impl_->fixed_keyframe_indexes_.end(), kfp.second->id_) !=
            impl_->fixed_keyframe_indexes_.end()) {
            continue;
        }
        auto cur_kf = kfp.second;
        if (!cur_kf->gps_inlier_) {
            continue;
        }
        if (cur_kf->bag_type_ == KeyFrameBagType::VALIDATION_BAGS) {
            continue;
        }

        SE3 obs_gps = cur_kf->gps_pose_;
        V6d noise_gps = cur_kf->gps_noise_;

        auto *e = new g2o::EdgeSE3Prior();
        e->setId(impl_->edge_id_++);
        e->setVertex(0, impl_->optimizer_.vertex(cur_kf->id_));
        e->setMeasurement(SE3ToG2OSE3Quat(obs_gps));

        M6d cov = M6d::Zero();
        cov(0, 0) = noise_gps[0] * noise_gps[0] * (1 / impl_->optimization_params_.gps_weight);
        cov(1, 1) = noise_gps[1] * noise_gps[1] * (1 / impl_->optimization_params_.gps_weight);
        cov(2, 2) = noise_gps[2] * noise_gps[2] * (1 / impl_->optimization_params_.gps_weight) *
                    impl_->optimization_params_.gps_height_noise_ratio *
                    impl_->optimization_params_.gps_height_noise_ratio;
        cov(3, 3) = noise_gps[3] * noise_gps[3] * (1 / impl_->optimization_params_.gps_weight);
        cov(4, 4) = noise_gps[4] * noise_gps[4] * (1 / impl_->optimization_params_.gps_weight);
        cov(5, 5) = noise_gps[5] * noise_gps[5] * (1 / impl_->optimization_params_.gps_weight);

        M6d info = cov.inverse();
        e->setInformation(info);
        e->setParameterId(0, 0);

        auto *rk = new g2o::RobustKernelCauchy;
        rk->setDelta(gps_th);
        e->setRobustKernel(rk);
        e->setParameterId(0, 0);

        impl_->optimizer_.addEdge(e);
        impl_->gps_edges_.push_back(e);
    }
}

void OptimizationStage2::AddDRFactors() {
    size_t dr_continous_frames = impl_->optimization_params_.dr_continous_num;
    double dr_th = impl_->optimization_params_.dr_chi2_th;
    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == KeyFrameBagType::VALIDATION_BAGS) {
            continue;
        }

        for (size_t i = 0; i < kfp.second.size(); i++) {
            for (size_t j = 1; j < dr_continous_frames; ++j) {
                if ((i + j) >= kfp.second.size()) break;
                auto pre_key_frame = kfp.second[i];
                auto cur_key_frame = kfp.second[i + j];

                SE3 obs_dr = pre_key_frame->dr_pose_.inverse() * cur_key_frame->dr_pose_;
                V6d noise_dr = cur_key_frame->dr_noise_;

                auto *e = new g2o::EdgeSE3();
                e->setId(impl_->edge_id_++);
                e->setVertex(0, impl_->optimizer_.vertex(pre_key_frame->id_));
                e->setVertex(1, impl_->optimizer_.vertex(cur_key_frame->id_));
                e->setMeasurement(SE3ToG2OSE3Quat(obs_dr));

                M6d cov = M6d::Zero();
                cov(0, 0) = noise_dr[0] * noise_dr[0] * (1 / impl_->optimization_params_.dr_weight);
                cov(1, 1) = noise_dr[1] * noise_dr[1] * (1 / impl_->optimization_params_.dr_weight);
                cov(2, 2) = noise_dr[2] * noise_dr[2] * (1 / impl_->optimization_params_.dr_weight);
                cov(3, 3) = noise_dr[3] * noise_dr[3] * (1 / impl_->optimization_params_.dr_weight);
                cov(4, 4) = noise_dr[4] * noise_dr[4] * (1 / impl_->optimization_params_.dr_weight);
                cov(5, 5) = noise_dr[5] * noise_dr[5] * (1 / impl_->optimization_params_.dr_yaw_weight);

                M6d info = cov.inverse();
                e->setInformation(info);

                if (impl_->optimization_params_.use_rk_dr) {
                    auto *rk = new g2o::RobustKernelCauchy;
                    rk->setDelta(dr_th);
                    e->setRobustKernel(rk);
                }

                impl_->optimizer_.addEdge(e);
                impl_->dr_edges_.push_back(e);
            }
        }
    }
}

void OptimizationStage2::AddMatchingFactors() {
    size_t num = impl_->optimization_params_.lidar_continous_num;
    double matching_th = impl_->optimization_params_.matching_chi2_th;

    LOG(INFO) << "adding matching";
    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == KeyFrameBagType::VALIDATION_BAGS) {
            continue;
        }

        for (size_t i = 0; i < kfp.second.size(); i++) {
            for (size_t j = 1; j < num; ++j) {
                if ((i + j) >= kfp.second.size()) {
                    break;
                }

                auto pre_key_frame = kfp.second[i];
                auto cur_key_frame = kfp.second[i + j];

                SE3 obs_matching = pre_key_frame->matching_pose_.inverse() * cur_key_frame->matching_pose_;
                V6d noise_matching = cur_key_frame->matching_noise_;

                auto *e = new g2o::EdgeSE3();
                e->setId(impl_->edge_id_++);
                e->setVertex(0, impl_->optimizer_.vertex(pre_key_frame->id_));
                e->setVertex(1, impl_->optimizer_.vertex(cur_key_frame->id_));
                e->setMeasurement(SE3ToG2OSE3Quat(obs_matching));
                M6d cov = M6d::Zero();
                cov(0, 0) = noise_matching[0] * noise_matching[0] * (1 / impl_->optimization_params_.matching_weight);
                cov(1, 1) = noise_matching[1] * noise_matching[1] * (1 / impl_->optimization_params_.matching_weight);
                cov(2, 2) = noise_matching[2] * noise_matching[2] * (1 / impl_->optimization_params_.matching_weight);
                cov(3, 3) = noise_matching[3] * noise_matching[3] * (1 / impl_->optimization_params_.matching_weight);
                cov(4, 4) = noise_matching[4] * noise_matching[4] * (1 / impl_->optimization_params_.matching_weight);
                cov(5, 5) = noise_matching[5] * noise_matching[5] * (1 / impl_->optimization_params_.matching_weight);
                M6d info = cov.inverse();
                e->setInformation(info);

                if (impl_->optimization_params_.use_rk_matching) {
                    auto *rk = new g2o::RobustKernelCauchy;
                    rk->setDelta(matching_th);
                    e->setRobustKernel(rk);
                }

                impl_->optimizer_.addEdge(e);
                impl_->matching_edges_.push_back(e);
            }
        }
    }
}

void OptimizationStage2::AddGlobalRotationFactors() {
    LOG(INFO) << "adding global rotation constraints";
    double global_rot_th = 1.0;
    double sigma_inv = 1.0 / (5 * M_PI / 180);

    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == KeyFrameBagType::VALIDATION_BAGS) {
            continue;
        }

        for (auto &kf : kfp.second) {
            if (std::find(impl_->fixed_keyframe_indexes_.begin(), impl_->fixed_keyframe_indexes_.end(), kf->id_) !=
                impl_->fixed_keyframe_indexes_.end()) {
                continue;
            }
            auto *e = new core::EdgeSE3RotPrior();
            e->setId(impl_->edge_id_++);
            e->setVertex(0, impl_->optimizer_.vertex(kf->id_));

            M2d info = M2d::Identity() * sigma_inv * sigma_inv;
            e->setInformation(info);

            auto *rk = new g2o::RobustKernelCauchy;
            rk->setDelta(global_rot_th);
            e->setRobustKernel(rk);

            impl_->optimizer_.addEdge(e);
            impl_->rot_edges_.push_back(e);
        }
    }
}

void OptimizationStage2::RemoveOutliers() {
    // 调整chi2阈值
    LOG(INFO) << "removing outliers";
    CollectOptimizationStatistics();

    if (impl_->if_use_nav_sat_) {
        int cnt_outlier = 0, cnt_inlier = 0;
        int gps_adj_iteration = 0;
        const double gps_inlier_ratio = 0.1;  // 要求大于这个比例的GPS是有效的
        double inlier_ratio = 0.0;
        impl_->gps_th_ = impl_->optimization_params_.gps_chi2_th;

        while (gps_adj_iteration < 3) {
            // gps statistics
            std::vector<double> gps_chi2;
            for (auto &edge : impl_->gps_edges_) {
                if (edge->chi2() > impl_->gps_th_) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
                gps_chi2.push_back(edge->chi2());
            }

            inlier_ratio = cnt_inlier / double(impl_->vertices_.size());
            if (inlier_ratio > gps_inlier_ratio) {
                break;
            } else {
                // enlarge gps th
                impl_->gps_th_ *= 2;
                LOG(INFO) << "inlier ratio not enough: " << inlier_ratio << ", enlarge gps th to " << impl_->gps_th_;
                gps_adj_iteration++;
            }
        }

        // set gps th
        for (auto &edge : impl_->gps_edges_) {
            edge->robustKernel()->setDelta(impl_->gps_th_);
        }

        if (inlier_ratio < gps_inlier_ratio) {
            LOG(WARNING) << "GPS inliers not enough: " << inlier_ratio << "<" << gps_inlier_ratio;
        }
        impl_->gps_inlier_ratio_ = inlier_ratio;

        LogAndReport("GPS无效/有效: " + std::to_string(cnt_outlier) + "/" + std::to_string(cnt_inlier) +
                     +", 阈值 = " + std::to_string(impl_->gps_th_));

        LOG(INFO) << "GPS: Outlier/Inlier in optimization: " << cnt_outlier << "/" << cnt_inlier
                  << ", gps_th = " << impl_->gps_th_;
        report_ += "S2: GPS inliers/outlier: " + std::to_string(cnt_inlier) + "/" + std::to_string(cnt_outlier) +
                   ", in/out=" + std::to_string(inlier_ratio) + "\n";
    }

    // 剔除outlier
    int cnt_outlier_removed = 0;
    auto remove_outlier = [&cnt_outlier_removed](g2o::OptimizableGraph::Edge *e) {
        if (e->chi2() > e->robustKernel()->delta()) {
            e->setLevel(1);
            cnt_outlier_removed++;
        } else {
            e->setRobustKernel(nullptr);
        }
    };

    auto remove_inlier_rb_kernel = [](g2o::OptimizableGraph::Edge *e) {
        if (e->chi2() <= e->robustKernel()->delta()) {
            e->setRobustKernel(nullptr);
        }
    };

    auto remove_rb_kernel = [](g2o::OptimizableGraph::Edge *e) { e->setRobustKernel(nullptr); };
    if (impl_->if_use_nav_sat_) {
        std::for_each(impl_->gps_edges_.begin(), impl_->gps_edges_.end(), remove_outlier);
        LogAndReport("GPS异常值：" + std::to_string(cnt_outlier_removed));
    }

    std::map<IdType, KFPtr> keyframes_map;
    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
            keyframes_map.insert({kf->id_, kf});
        }
    }

    for (auto &e : impl_->gps_edges_) {
        if (e->level() == 0 && impl_->if_use_nav_sat_) {
            keyframes_map[e->vertex(0)->id()]->gps_inlier_ = true;
        } else {
            keyframes_map[e->vertex(0)->id()]->gps_inlier_ = false;
        }
    }

    if (impl_->optimization_params_.use_rk_dr) {
        std::for_each(impl_->dr_edges_.begin(), impl_->dr_edges_.end(), remove_inlier_rb_kernel);
    }

    if (impl_->optimization_params_.use_rk_matching) {
        cnt_outlier_removed = 0;
        std::for_each(impl_->matching_edges_.begin(), impl_->matching_edges_.end(), remove_outlier);
        LogAndReport("激光匹配异常值：" + std::to_string(cnt_outlier_removed));
    }

    cnt_outlier_removed = 0;
    std::for_each(impl_->loop_edges_.begin(), impl_->loop_edges_.end(), remove_outlier);
    LOG(INFO) << "loop edges outliers: " << cnt_outlier_removed << "/" << impl_->loop_edges_.size();
}

void OptimizationStage2::Solve() {
    impl_->optimizer_.setVerbose(false);
    impl_->optimizer_.initializeOptimization();
    if (reset_iteration_) {
        impl_->optimizer_.optimize(iteration_);
    } else {
        impl_->optimizer_.optimize(1000);
    }

    /// fetch results
    std::map<IdType, KFPtr> keyframes_map;
    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
            keyframes_map.insert({kf->id_, kf});
        }
    }

    for (auto *v : impl_->vertices_) {
        keyframes_map[v->id()]->optimized_pose_stage_2_ = SE3(v->estimate().matrix());
    }
}

template <typename T>
std::string print_info(const std::vector<T> &edges, double th) {
    std::vector<double> chi2;
    for (auto &edge : edges) {
        if (edge->level() == 0) {
            chi2.push_back(edge->chi2());
        }
    }

    std::sort(chi2.begin(), chi2.end());
    double ave_chi2 = std::accumulate(chi2.begin(), chi2.end(), 0.0) / chi2.size();
    boost::format fmt("数量: %d, 均值: %f, 中位数: %f, 0.1分位: %f, 0.9分位: %f, 最大值: %f, 阈值: %f\n");
    if (!chi2.empty()) {
        std::string str = (fmt % chi2.size() % ave_chi2 % chi2[chi2.size() / 2] % chi2[int(chi2.size() * 0.1)] %
                           chi2[int(chi2.size() * 0.9)] % chi2.back() % th)
                              .str();
        return str;
    }
    return std::string("");
}

void OptimizationStage2::CollectOptimizationStatistics() {
    // print statistics
    std::string gnss_repo;
    if (impl_->if_use_nav_sat_) {
        gnss_repo = std::string("GNSS ") + print_info(impl_->gps_edges_, impl_->gps_th_);
    }

    std::string match_repo =
        std::string("Match ") + print_info(impl_->matching_edges_, impl_->optimization_params_.matching_chi2_th);
    std::string dr_repo = std::string("DR ") + print_info(impl_->dr_edges_, impl_->optimization_params_.dr_chi2_th);
    std::string rot_repo = std::string("Rotation ") + print_info(impl_->rot_edges_, 0);
    std::string loop_repo =
        std::string("Loop closure ") + print_info(impl_->loop_edges_, impl_->optimization_params_.loop_chi2_th);
    std::string height_repo = std::string("Heights ") + print_info(impl_->height_edges_, 0);
    // manual statistics
    std::vector<double> manual_chi2;
    for (auto &edge : impl_->manual_edges_) {
        if (edge->level() == 0) {
            manual_chi2.push_back(edge->chi2());
        }
    }
    std::sort(manual_chi2.begin(), manual_chi2.end());
    double sum_manual_chi2 = std::accumulate(manual_chi2.begin(), manual_chi2.end(), 0.0);
    double ave_manual_chi2 = sum_manual_chi2 / manual_chi2.size();
    std::string manual_repo = std::string("Manual ") + print_info(impl_->manual_edges_, ave_manual_chi2);

    LogAndReport(gnss_repo);
    LogAndReport(match_repo);
    LogAndReport(dr_repo);
    LogAndReport(rot_repo);
    LogAndReport(loop_repo);
    LogAndReport(height_repo);
    LogAndReport(manual_repo);
}

void OptimizationStage2::AddLoopClosureFactors() {
    bool is_gps_good = impl_->gps_status_ == BagGNSSStatusType::BAG_GNSS_POS_HEAD_VALID_SYNC;

    const double loop_noise_pos = 0.2;
    const double loop_noise_ang = 2.5 / 180.0 * M_PI;
    const double loop_th = impl_->optimization_params_.loop_chi2_th;

    for (auto &lc : impl_->loop_candidates_) {
        auto kf0 = impl_->keyframes_map_by_id_.at(lc.kfid_first);
        auto kf1 = impl_->keyframes_map_by_id_.at(lc.kfid_second);

        auto *e = new g2o::EdgeSE3();
        e->setId(impl_->edge_id_++);
        e->setVertex(0, impl_->optimizer_.vertex(lc.kfid_first));
        e->setVertex(1, impl_->optimizer_.vertex(lc.kfid_second));
        e->setMeasurement(SE3ToG2OSE3Quat(lc.Tij));
        M6d cov = M6d::Zero();
        cov(0, 0) = loop_noise_pos * loop_noise_pos * (1 / impl_->optimization_params_.loop_weight);
        cov(1, 1) = loop_noise_pos * loop_noise_pos * (1 / impl_->optimization_params_.loop_weight);
        cov(2, 2) = loop_noise_pos * loop_noise_pos * (1 / impl_->optimization_params_.loop_weight);
        cov(3, 3) = loop_noise_ang * loop_noise_ang * (1 / impl_->optimization_params_.loop_weight);
        cov(4, 4) = loop_noise_ang * loop_noise_ang * (1 / impl_->optimization_params_.loop_weight);
        cov(5, 5) = loop_noise_ang * loop_noise_ang * (1 / impl_->optimization_params_.loop_weight);
        M6d info = cov.inverse();
        e->setInformation(info);

        auto *rk = new g2o::RobustKernelCauchy;
        rk->setDelta(loop_th);
        e->setRobustKernel(rk);

        impl_->optimizer_.addEdge(e);
        impl_->loop_edges_.push_back(e);
    }

    LOG(INFO) << "loop factors: " << impl_->loop_edges_.size();
}

void OptimizationStage2::AddHeightFactors() {
    if (impl_->optimization_params_.with_height == false) {
        LogAndReport("由于配置文件设定，不加入高度约束");
        return;
    }

    if (impl_->gps_status_ == BagGNSSStatusType::BAG_GNSS_POS_HEAD_VALID_SYNC && impl_->gps_inlier_ratio_ > 0.1 &&
        impl_->if_use_nav_sat_) {
        LogAndReport("存在GPS有效值，不加入高度约束");
        return;
    }

    LogAndReport("增加高度约束");
    std::vector<double> ave_height_global;  // 全局高度值
    for (auto &v : impl_->vertices_) {
        ave_height_global.push_back(v->estimate().translation()[2]);
    }
    if (ave_height_global.empty()) {
        return;
    }

    const double height_noise = 0.1;
    double h = ave_height_global[int(ave_height_global.size() / 2)];

    for (auto &tp : impl_->keyframes_) {
        if (tp.second[0]->bag_type_ == KeyFrameBagType::VALIDATION_BAGS) {
            continue;
        }

        for (auto kf : tp.second) {
            int id = kf->id_;
            auto *e = new core::EdgeSE3HeightPrior();
            e->setId(impl_->edge_id_++);
            e->setVertex(0, impl_->optimizer_.vertex(id));

            M1d cov = M1d::Identity() * height_noise * height_noise;
            e->setInformation(cov.inverse());
            e->setMeasurement(h);
            impl_->optimizer_.addEdge(e);
            impl_->height_edges_.push_back(e);
        }
    }

    LOG(INFO) << "height factors: " << impl_->height_edges_.size() << ", mean h = " << h;
}

bool OptimizationStage2::UpdateMapHeight(double_t map_height) {
    double_t average_height = 0.0;

    if (common::GpsLowAccuracy(impl_->gps_status_)) {
        double_t count = 1.0;
        for (auto &kfp : impl_->keyframes_) {
            for (auto kf : kfp.second) {
                if (kf->gps_noise_(0) < 1e-7) {
                    average_height =
                        (kf->optimized_pose_stage_2_.translation()[2] - average_height) / count + average_height;
                    ++count;
                }
            }
        }

        if (std::fabs(average_height) > 10.0 && count > 3.0) {
            map_height = average_height + map_height;

            for (auto &kf : impl_->keyframes_map_by_id_) {
                kf.second->gps_pose_.translation()[2] -= average_height;
                kf.second->matching_pose_.translation()[2] -= average_height;
                kf.second->dr_pose_.translation()[2] -= average_height;
                kf.second->optimized_pose_stage_1_.translation()[2] -= average_height;
                kf.second->optimized_pose_stage_2_.translation()[2] -= average_height;
            }

            for (auto &kfp : impl_->keyframes_) {
                for (auto kf : kfp.second) {
                    kf->gps_pose_.translation()[2] -= average_height;
                    kf->matching_pose_.translation()[2] -= average_height;
                    kf->dr_pose_.translation()[2] -= average_height;
                    kf->optimized_pose_stage_1_.translation()[2] -= average_height;
                    kf->optimized_pose_stage_2_.translation()[2] -= average_height;
                }
            }

            return true;
        }
    }
    return false;
}

void OptimizationStage2::LoadFixedKeyframesIndexes(const std::vector<int> &keyframes_indexes) {
    impl_->fixed_keyframe_indexes_ = keyframes_indexes;
}

void OptimizationStage2::LoadFixedKeyframesIndex(const int keyframe_index) {
    impl_->fixed_keyframe_indexes_.emplace_back(keyframe_index);
}

void OptimizationStage2::LoadKeyframesFromQuickDebug(const std::map<IdType, common::KFPtr> &kfs) {
    impl_->keyframes_map_by_id_ = kfs;

    get_keyframes_from_quick_debug_ = true;
}

void OptimizationStage2::SetOptimizationParams(const OptimizationParams &params) {
    impl_->optimization_params_ = params;
}

void OptimizationStage2::AddFixFactors() {
    if (impl_->fixed_keyframe_indexes_.empty()) {
        return;
    }
    // 添加边：人工 FIX 约束
    for (auto &i : impl_->fixed_keyframe_indexes_) {
        auto cur_kf = impl_->keyframes_map_by_id_[i];

        if (cur_kf->bag_type_ == KeyFrameBagType::VALIDATION_BAGS) {
            continue;
        }

        SE3 obs_opt = cur_kf->optimized_pose_stage_2_;

        g2o::EdgeSE3Prior *e = new g2o::EdgeSE3Prior();
        e->setId(impl_->edge_id_++);
        e->setVertex(0, impl_->optimizer_.vertex(cur_kf->id_));
        e->setMeasurement(SE3ToG2OSE3Quat(obs_opt));

        M6d info = M6d::Identity() * 1e6;
        e->setInformation(info);
        e->setParameterId(0, 0);

        impl_->optimizer_.addEdge(e);
        impl_->manual_edges_.push_back(e);
    }
}

void OptimizationStage2::SetIteration(uint8_t num) {
    reset_iteration_ = true;
    iteration_ = num;
}

void OptimizationStage2::UpdateDb() {
    LOG(INFO) << "updating db";
    if (run_mode_ == PipelineContext::RunMode::PIPELINE) {
        io::DB_IO db_io(local_data_path_ + "map.db");
        db_io.UpdateKeyFramePoses(impl_->keyframes_map_by_id_);
        if (!db_io.WriteOriginPointInformationToDB(impl_->origin_info_)) {
            LOG(ERROR) << "Write origin point information to db failed!!!";
        }
    } else {
        io::DB_IO db_io(local_db_path_ + "map.db");
        db_io.UpdateKeyFramePoses(impl_->keyframes_map_by_id_);
        if (!db_io.WriteOriginPointInformationToDB(impl_->origin_info_)) {
            LOG(ERROR) << "Write origin point information to db failed!!!";
        }
    }
}

void OptimizationStage2::SavePcd() {
    LOG(INFO) << "saving pcd";
    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == common::KeyFrameBagType::VALIDATION_BAGS) {
            continue;
        }

        std::string db_path;
        if (run_mode_ == PipelineContext::RunMode::PIPELINE) {
            db_path = local_data_path_ + "map.db";
        } else {
            db_path = local_db_path_ + "map.db";
        }

        // if (kfp.second[0]->bag_type_ == common::KeyFrameBagType::VALIDATION_BAGS) {
        //     db_path = local_data_path_ + "val.db";
        // }

        int max_pcd_size = 2000;
        size_t num = kfp.second.size() / max_pcd_size;
        for (size_t i = 0; i < num; ++i) {
            std::vector<std::shared_ptr<common::KeyFrame>> temp(kfp.second.begin() + i * max_pcd_size,
                                                                kfp.second.begin() + (i + 1) * max_pcd_size);
            io::SavePCDWithPose(
                local_data_path_ + "optimization2_" + std::to_string(kfp.first) + "_" + std::to_string(i) + ".pcd",
                db_path, temp, io::SaveKeyframePathType::OPTI_PATH_STAGE_2);
        }
        std::vector<std::shared_ptr<common::KeyFrame>> temp_vc(kfp.second.begin() + max_pcd_size * num,
                                                               kfp.second.end());
        io::SavePCDWithPose(
            local_data_path_ + "optimization2_" + std::to_string(kfp.first) + "_" + std::to_string(num) + ".pcd",
            db_path, temp_vc, io::SaveKeyframePathType::OPTI_PATH_STAGE_2);
    }
}

}  // namespace mapping::pipeline
