//
// Created by gaoxiang on 2020/9/16.
//

#include "pipeline/optimization_s1/optimization_s1.h"
#include "common/mapping_math.h"
#include "core/mt_search/multi_thread_search.h"
#include "core/optimization/calib_edge.h"
#include "io/file_io.h"
#include "pipeline/optimization_s1/optimization_s1_impl.h"

#include <glog/logging.h>
#include <numeric>

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <boost/format.hpp>

using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;

namespace mapping::pipeline {

using namespace mapping::common;

/// SE3 转g2o的SE3Quat
inline g2o::SE3Quat SE3ToG2OSE3Quat(const SE3 &T) { return g2o::SE3Quat(T.so3().matrix(), T.translation()); }

/// 打印优化信息
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
    return std::string();
}

OptimizationStage1::OptimizationStage1(const io::YAML_IO &yaml_file, RunMode run_mode)
    : yaml_file_(yaml_file), impl_(new OptimizationStage1Impl), PipelineContext(run_mode) {
    context_name_ = "OptimizationStage1";
}

OptimizationStage1::~OptimizationStage1() noexcept { LOG(INFO) << "Opt s1 deconstrcuted"; }

bool OptimizationStage1::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }
    impl_->origin_info_.map_origin_x = yaml_file_.GetValue<double>("map_origin_param", "map_origin_x");
    impl_->origin_info_.map_origin_y = yaml_file_.GetValue<double>("map_origin_param", "map_origin_y");
    impl_->origin_info_.map_origin_z = yaml_file_.GetValue<double>("map_origin_param", "map_origin_z");
    impl_->origin_info_.map_origin_zone = yaml_file_.GetValue<int>("map_origin_param", "map_origin_zone");
    impl_->origin_info_.is_southern = yaml_file_.GetValue<bool>("map_origin_param", "is_southern");

    impl_->optimization_params_.LoadFromYAML(yaml_file_);
    impl_->gps_status_ = common::BagGNSSStatusType(yaml_file_.GetValue<int>("gps_status"));
    impl_->gps_th_ = impl_->optimization_params_.gps_chi2_th;
    impl_->save_debug_pcd_ = yaml_file_.GetValue<bool>("save_debug_pcd");
    impl_->is_open_calib_lidar_gnss_ = yaml_file_.GetValue<bool>("is_open_calib_lidar_gnss");
    impl_->if_use_nav_sat_ = yaml_file_.GetValue<bool>("use_nav_sat");

    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");
    local_db_path_ = yaml_file_.GetValue<std::string>("local_db_path");

    if (!io::LoadKeyframes(local_data_path_ + "keyframes.txt", impl_->keyframes_)) {
        LogAndReport("\\textbf{错误}：无法读取关键帧文件");
        return false;
    }

    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
            impl_->keyframes_map_.insert({kf->id_, kf});
        }
    }

    return true;
}

bool OptimizationStage1::Start() {
    LOG(INFO) << "Optimization stage 1";
    SetStatus(ContextStatus::WORKING);
    CalculateInitialValue();

    BuildProblem();

    /// 第一轮
    LogAndReport("求解第一次优化");
    impl_->optimizer_.save((local_data_path_ + "s1_init.g2o").c_str());
    Solve(false);

    /// 去outlier
    RemoveOutliers();

    /// 对lidar和gnss进行再标定
    if (impl_->is_open_calib_lidar_gnss_ && impl_->if_use_nav_sat_) {
        CalibLidarGNSS();
    }

    AddHeightFactors();

    /// 第二轮
    LogAndReport("求解第二次优化");
    Solve(true);
    impl_->optimizer_.save((local_data_path_ + "s1_second.g2o").c_str());

    // 如果整个地图GPS不好，而且优化轨迹离原点较远，就进行平移
    MoveToOriginInGpsDenied();

    // 尝试对轨迹进行连接
    ConnectTrajectories();

    /// 第三轮
    LogAndReport("求解第三次优化");
    Solve(true);

    /// 打印信息
    CollectOptimizationStatistics();

    SaveResults();

    LogAndReport("优化步骤1：完成");
    SetStatus(ContextStatus::SUCCEED);
    return true;
}

void OptimizationStage1::ResetToInitialValue() {
    LOG(INFO) << "reset to initial value";
    std::map<IdType, KFPtr> keyframes_map;
    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
            keyframes_map.insert({kf->id_, kf});
        }
    }

    for (auto *v : impl_->vertices_) {
        v->setEstimate(SE3ToG2OSE3Quat(keyframes_map[v->id()]->optimized_pose_stage_1_));
    }
}

void OptimizationStage1::SaveResults() {
    /// 存储第一轮优化结果
    if (impl_->is_open_calib_lidar_gnss_) {
        for (auto &kfp : impl_->keyframes_) {
            for (auto &this_kf : kfp.second) {
                this_kf->gps_pose_ = this_kf->gps_pose_ * impl_->TGL_;
            }
        }
    }
    LOG(INFO) << "saving path";
    io::RemoveIfExist(local_data_path_ + "optimization1.txt");
    io::SaveKeyframeSinglePath(local_data_path_ + "optimization1.txt", impl_->keyframes_,
                               io::SaveKeyframePathType::OPTI_PATH_STAGE_1);

    io::RemoveIfExist(local_data_path_ + "matching_path.txt");
    io::SaveKeyframeSinglePath(local_data_path_ + "matching_path.txt", impl_->keyframes_,
                               io::SaveKeyframePathType::MATCHING_PATH);
    io::RemoveIfExist(local_data_path_ + "dr_path.txt");
    io::SaveKeyframeSinglePath(local_data_path_ + "dr_path.txt", impl_->keyframes_, io::SaveKeyframePathType::DR_PATH);
    io::RemoveIfExist(local_data_path_ + "gps_path.txt");
    io::SaveKeyframeSinglePath(local_data_path_ + "gps_path.txt", impl_->keyframes_,
                               io::SaveKeyframePathType::GPS_PATH);

    LOG(INFO) << "saving keyframes";
    io::RemoveIfExist(local_data_path_ + "keyframes.txt");
    io::SaveKeyframePose(local_data_path_ + "keyframes.txt", impl_->keyframes_);

    if (impl_->save_debug_pcd_) {
        LOG(INFO) << "saving pcd";
        for (auto &kfp : impl_->keyframes_) {
            if (kfp.second[0]->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
                std::string db_path = local_data_path_ + "map.db";
                io::SavePCDWithPose(local_data_path_ + "optimization1_" + std::to_string(kfp.first) + ".pcd", db_path,
                                    kfp.second, io::SaveKeyframePathType::OPTI_PATH_STAGE_1);
            }
        }
    }

    impl_->optimizer_.save((local_data_path_ + "s1_final.g2o").c_str());

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        SaveGemReport(local_data_path_ + gem_report_name_);
    }
}

void OptimizationStage1::CalculateInitialValue() {
    if (impl_->gps_status_ == common::BagGNSSStatusType::BAG_GNSS_NOT_EXIST || !impl_->if_use_nav_sat_) {
        CalculateInitialValueWithoutGps();
    } else {
        CalculateInitialValueWithGps();
    }

    if (impl_->save_debug_pcd_) {
        LOG(INFO) << "saving initial point clouds.";
        for (auto &kfp : impl_->keyframes_) {
            std::string db_path;
            if (kfp.second[0]->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
                db_path = local_data_path_ + "map.db";
            } else {
                db_path = local_data_path_ + "val.db";
            }

            io::SavePCDWithPose(local_data_path_ + "initial_" + std::to_string(kfp.first) + ".pcd", db_path, kfp.second,
                                io::SaveKeyframePathType::OPTI_PATH_STAGE_1);
        }
    }
}

void OptimizationStage1::CalculateInitialValueWithGps() {
    LogAndReport("使用GPS估计初始轨迹");
    // 使用小型pose graph设立初始值
    auto *solver =
        new g2o::OptimizationAlgorithmDogleg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    auto *offset_value = new g2o::ParameterSE3Offset();
    offset_value->setId(0);
    optimizer.addParameter(offset_value);

    int edge_id = 1;
    std::map<KFPtr, g2o::VertexSE3 *> kf_to_vert;

    /// vertex and gps edges
    // 在计算初始值时，不给GPS很高的权重以防止带偏，information矩阵乘以此倍率
    std::vector<g2o::EdgeSE3Prior *> gps_edges;
    constexpr double info_initial_ratio = 1e-2;

    int cnt_set_by_gnss = 0;
    int cnt_set_by_inferring = 0;
    int cnt_set_by_matching = 0;

    for (auto &kfp : impl_->keyframes_) {
        bool has_closest_gps_pose = false;
        common::KFPtr closest_gps_kf = nullptr;

        for (auto &kf : kfp.second) {
            auto *v = new g2o::VertexSE3();
            v->setId(kf->id_);
            optimizer.addVertex(v);
            kf->SetGpsNoiseFromStatus();

            /// 初始值的初始值 哈哈哈
            if (GpsUsable(kf->gps_status_)) {
                SE3 pose = kf->gps_pose_;
                if (kf->heading_valid_ == false) {
                    // estimate heading via closest dr
                    if (has_closest_gps_pose) {
                        SE3 estimated_pose =
                            closest_gps_kf->gps_pose_ * closest_gps_kf->matching_pose_.inverse() * kf->matching_pose_;
                        pose.so3() = estimated_pose.so3();
                    } else {
                        pose.so3() = kf->matching_pose_.so3();
                    }
                }

                v->setEstimate(SE3ToG2OSE3Quat(pose));
                has_closest_gps_pose = true;
                closest_gps_kf = kf;
                cnt_set_by_gnss++;
            } else {
                /// 使用DR或matching进行递推
                if (has_closest_gps_pose) {
                    SE3 estimated_pose =
                        closest_gps_kf->gps_pose_ * closest_gps_kf->matching_pose_.inverse() * kf->matching_pose_;
                    v->setEstimate(SE3ToG2OSE3Quat(estimated_pose));
                    cnt_set_by_inferring++;
                } else {
                    v->setEstimate(SE3ToG2OSE3Quat(kf->matching_pose_));
                    cnt_set_by_matching++;
                }
            }

            /// GPS factor
            if (GpsUsable(kf->gps_status_)) {
                V6d noise_gps = kf->gps_noise_;

                // GPS constraint
                auto *e = new g2o::EdgeSE3Prior();
                e->setId(edge_id++);
                e->setVertex(0, v);
                e->setMeasurement(SE3ToG2OSE3Quat(kf->gps_pose_));

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
                e->setInformation(info * info_initial_ratio);
                e->setParameterId(0, 0);

                auto *rk = new g2o::RobustKernelCauchy();
                rk->setDelta(0.5);
                e->setRobustKernel(rk);

                optimizer.addEdge(e);
                gps_edges.push_back(e);
            }

            kf_to_vert.insert({kf, v});
        }
    }

    LogAndReport("初始值由GNSS设置：" + std::to_string(cnt_set_by_gnss) + "，由递推：" +
                 std::to_string(cnt_set_by_inferring) + "，由激光匹配：" + std::to_string(cnt_set_by_matching));

    // matching edges
    std::vector<g2o::EdgeSE3 *> matching_edges;
    for (auto &kfp : impl_->keyframes_) {
        for (size_t i = 0; i < kfp.second.size() - 1; i++) {
            int j = 1;

            auto pre_key_frame = kfp.second[i];
            auto cur_key_frame = kfp.second[i + j];

            SE3 relative_motion;
            V6d relative_noise;
            if (pre_key_frame->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
                relative_motion = pre_key_frame->matching_pose_.inverse() * cur_key_frame->matching_pose_;
                relative_noise = cur_key_frame->matching_noise_;
            } else {
                relative_motion = pre_key_frame->dr_pose_.inverse() * cur_key_frame->dr_pose_;
                relative_noise = cur_key_frame->dr_noise_;
                if (impl_->valid_bag_use_matching_pose_) {
                    relative_motion = pre_key_frame->matching_pose_.inverse() * cur_key_frame->matching_pose_;
                    relative_noise = cur_key_frame->matching_noise_;
                }
            }

            auto *e = new g2o::EdgeSE3();
            e->setId(edge_id++);
            e->setVertex(0, kf_to_vert[pre_key_frame]);
            e->setVertex(1, kf_to_vert[cur_key_frame]);
            e->setMeasurement(SE3ToG2OSE3Quat(relative_motion));

            M6d cov = M6d::Zero();
            cov(0, 0) = relative_noise[0] * relative_noise[0] * (1 / impl_->optimization_params_.matching_weight);
            cov(1, 1) = relative_noise[1] * relative_noise[1] * (1 / impl_->optimization_params_.matching_weight);
            cov(2, 2) = relative_noise[2] * relative_noise[2] * (1 / impl_->optimization_params_.matching_weight);
            cov(3, 3) = relative_noise[3] * relative_noise[3] * (1 / impl_->optimization_params_.matching_weight);
            cov(4, 4) = relative_noise[4] * relative_noise[4] * (1 / impl_->optimization_params_.matching_weight);
            cov(5, 5) = relative_noise[5] * relative_noise[5] * (1 / impl_->optimization_params_.matching_weight);

            M6d info = cov.inverse();
            e->setInformation(info);

            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(0.5);
            e->setRobustKernel(rk);

            optimizer.addEdge(e);
            matching_edges.push_back(e);
        }
    }

    optimizer.save((local_data_path_ + "initial_before.g2o").c_str());
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(50);

    // 使用DR+matching组合来作为初值
    for (auto &kfv : kf_to_vert) {
        kfv.first->optimized_pose_stage_1_ = SE3(kfv.second->estimate().matrix());
    }

    LogAndReport("初始GPS: " + print_info(gps_edges, 0.0));
    LogAndReport("初始相对运动: " + print_info(matching_edges, 0.0));

    optimizer.save((local_data_path_ + "initial_after.g2o").c_str());
    io::SaveKeyframeSinglePath(local_data_path_ + "initial.txt", impl_->keyframes_,
                               io::SaveKeyframePathType::OPTI_PATH_STAGE_1);
}

void OptimizationStage1::CalculateInitialValueWithoutGps() {
    // 无GPS时，直接用matching pose作为初始值
    LOG(INFO) << "calculate initial value without gps";
    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
            kf->optimized_pose_stage_1_ = kf->matching_pose_;
        }
    }
}

void OptimizationStage1::BuildProblem() {
    auto *solver =
        new g2o::OptimizationAlgorithmDogleg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

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

    // 选配部分
    ///  Stage 1里不使用高度约束，以防和GPS冲突  FIXME ? 重新考虑
    if (impl_->optimization_params_.with_global_rotation) {
        AddGlobalRotationFactors();
    }
}

void OptimizationStage1::AddVertex() {
    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
            auto *v = new g2o::VertexSE3();
            v->setEstimate(SE3ToG2OSE3Quat(kf->optimized_pose_stage_1_));
            v->setId(kf->id_);
            impl_->optimizer_.addVertex(v);
            impl_->vertices_.emplace_back(v);
        }
    }
}

void OptimizationStage1::AddGpsFactors() {
    LOG(INFO) << "adding gps factors, using height noise ratio: " << impl_->optimization_params_.gps_height_noise_ratio;
    double gps_th = impl_->optimization_params_.gps_chi2_th;
    for (auto &kfp : impl_->keyframes_) {
        for (auto &cur_kf : kfp.second) {
            cur_kf->SetGpsNoiseFromStatus();

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

            auto *rk = new g2o::RobustKernelCauchy;
            rk->setDelta(gps_th);
            e->setRobustKernel(rk);
            e->setParameterId(0, 0);

            impl_->optimizer_.addEdge(e);
            impl_->gps_edges_.push_back(e);
        }
    }
}

void OptimizationStage1::AddDRFactors() {
    size_t dr_continous_frames = impl_->optimization_params_.dr_continous_num;
    double dr_th = impl_->optimization_params_.dr_chi2_th;
    for (auto &kfp : impl_->keyframes_) {
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

void OptimizationStage1::AddMatchingFactors() {
    size_t num = impl_->optimization_params_.lidar_continous_num;
    double matching_th = impl_->optimization_params_.matching_chi2_th;

    LOG(INFO) << "adding matching factors";
    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == KeyFrameBagType::VALIDATION_BAGS && !impl_->valid_bag_use_matching_pose_) {
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

void OptimizationStage1::AddGlobalRotationFactors() {
    LOG(INFO) << "adding global rotation constraints";
    double global_rot_th = 1.0;
    double sigma_inv = 1.0 / (5 * M_PI / 180);

    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
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

void OptimizationStage1::RemoveOutliers() {
    std::string gnss_repo;
    if (impl_->if_use_nav_sat_) {
        // 调整chi2阈值
        int cnt_outlier = 0, cnt_inlier = 0;
        int gps_adj_iteration = 0;
        const double gps_inlier_ratio = 0.1;  // 要求大于这个比例的GPS是有效的
        double inlier_ratio = 0.0;
        impl_->gps_th_ = impl_->optimization_params_.gps_chi2_th;

        // set gps th
        LOG(INFO) << "adjusting gps th";
        while (gps_adj_iteration < 3) {
            // gps statistics
            cnt_inlier = cnt_outlier = 0;
            std::vector<double> gps_chi2;
            for (auto &edge : impl_->gps_edges_) {
                if (edge->chi2() > impl_->gps_th_) {
                    cnt_outlier++;
                } else {
                    cnt_inlier++;
                }
                gps_chi2.push_back(edge->chi2());
            }

            inlier_ratio = cnt_inlier / double(cnt_inlier + cnt_outlier);
            if (inlier_ratio > gps_inlier_ratio) {
                break;
            } else {
                // enlarge gps th
                impl_->gps_th_ *= 2;
                LOG(INFO) << "inlier ratio not enough: " << inlier_ratio << ", enlarge gps th to " << impl_->gps_th_;
                gps_adj_iteration++;
            }
        }

        for (auto &edge : impl_->gps_edges_) {
            edge->robustKernel()->setDelta(impl_->gps_th_);
        }

        impl_->gps_inlier_ratio_ = inlier_ratio;
        if (inlier_ratio < gps_inlier_ratio) {
            LogAndReport("\\textbf{警告}：GPS有效值较低，有效比例：" + std::to_string(inlier_ratio));
        }

        // count good gps signals
        int cnt_good_gps = 0;
        for (auto &kfp : impl_->keyframes_) {
            for (auto &kf : kfp.second) {
                if (kf->gps_status_ == GpsStatusType::GNSS_FIXED_SOLUTION) {
                    cnt_good_gps++;
                }
            }
        }

        LogAndReport("GPS无效/有效/标称有效: " + std::to_string(cnt_outlier) + "/" + std::to_string(cnt_inlier) + "/" +
                     std::to_string(cnt_good_gps) + ", 阈值 = " + std::to_string(impl_->gps_th_));

        gnss_repo = std::string("GNSS ") + print_info(impl_->gps_edges_, impl_->gps_th_);
    }

    std::string match_repo =
        std::string("Match ") + print_info(impl_->matching_edges_, impl_->optimization_params_.matching_chi2_th);
    std::string dr_repo = std::string("DR ") + print_info(impl_->dr_edges_, impl_->optimization_params_.dr_chi2_th);
    std::string rot_repo = std::string("Rotation ") + print_info(impl_->rot_edges_, 0);

    LogAndReport("第一轮优化：");
    LogAndReport(gnss_repo);
    LogAndReport(match_repo);
    LogAndReport(dr_repo);
    LogAndReport(rot_repo);

    // 如果误差大，则保留rk
    auto remove_inlier_rb_kernel = [](g2o::OptimizableGraph::Edge *e) {
        if (e->chi2() <= e->robustKernel()->delta()) {
            e->setRobustKernel(nullptr);
        }
    };

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

    if (impl_->if_use_nav_sat_) {
        std::vector<double> gps_chi2;
        std::vector<V6d> gps_res;
        for (auto &edge : impl_->gps_edges_) {
            gps_chi2.push_back(edge->chi2());
            gps_res.push_back(edge->error());
        }

        io::SaveGpsError(local_data_path_ + "gps_error.txt", gps_chi2);
        io::SaveGpsError(local_data_path_ + "gps_res.txt", gps_res);

        auto remove_rb_kernel = [](g2o::OptimizableGraph::Edge *e) { e->setRobustKernel(nullptr); };

        LOG(INFO) << "removing outliers";
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
}

void OptimizationStage1::Solve(bool fetch_results) {
    LOG(INFO) << "solving optimization";
    impl_->optimizer_.setVerbose(false);
    impl_->optimizer_.initializeOptimization();
    impl_->optimizer_.optimize(1000);

    /// fetch results
    if (fetch_results) {
        std::map<IdType, KFPtr> keyframes_map;
        for (auto &kfp : impl_->keyframes_) {
            for (auto &kf : kfp.second) {
                keyframes_map.insert({kf->id_, kf});
            }
        }

        for (auto *v : impl_->vertices_) {
            keyframes_map[v->id()]->optimized_pose_stage_1_ = SE3(v->estimate().matrix());
        }
    }
}

void OptimizationStage1::CollectOptimizationStatistics() {
    // print statistics
    std::string gnss_repo;
    if (impl_->if_use_nav_sat_) {
        gnss_repo = std::string("GNSS ") + print_info(impl_->gps_edges_, impl_->gps_th_);
    }
    std::string match_repo =
        std::string("Match ") + print_info(impl_->matching_edges_, impl_->optimization_params_.matching_chi2_th);
    std::string dr_repo = std::string("DR ") + print_info(impl_->dr_edges_, impl_->optimization_params_.dr_chi2_th);
    std::string rot_repo = std::string("Rotation ") + print_info(impl_->rot_edges_, 0);
    std::string height_repo = std::string("Heights ") + print_info(impl_->height_edges_, 0);

    LogAndReport(gnss_repo);
    LogAndReport(match_repo);
    LogAndReport(dr_repo);
    LogAndReport(rot_repo);
    LogAndReport(height_repo);
}

void OptimizationStage1::CalibLidarGNSS() {
    if (impl_->gps_inlier_ratio_ < 0.1) {
        LogAndReport("GPS正常比例低，放弃标定Lidar/GNSS");
        return;
    }

    LogAndReport("重新标定Lidar/GNSS");
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver =
        new g2o::OptimizationAlgorithmDogleg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    g2o::VertexSE3 *vertex_pose = new g2o::VertexSE3();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Eigen::Isometry3d::Identity());
    optimizer.addVertex(vertex_pose);

    // edges
    int index = 1;
    std::vector<core::CalibEdge *> edges;
    double calib_th = 0.01;

    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
            for (size_t i = 0; i < kfp.second.size() - 1; ++i) {
                auto &this_kf = kfp.second[i];
                auto &next_kf = kfp.second[i + 1];

                if (this_kf->gps_inlier_ && next_kf->gps_inlier_ &&
                    this_kf->gps_status_ == GpsStatusType::GNSS_FIXED_SOLUTION &&
                    next_kf->gps_status_ == GpsStatusType::GNSS_FIXED_SOLUTION && this_kf->heading_valid_ &&
                    next_kf->heading_valid_) {
                    SE3 this_kf_matching_pose = GetPlanePose(this_kf->matching_pose_);
                    SE3 next_kf_matching_pose = GetPlanePose(next_kf->matching_pose_);
                    SE3 this_kf_gps_pose = GetPlanePose(this_kf->gps_pose_);
                    SE3 next_kf_gps_pose = GetPlanePose(next_kf->gps_pose_);
                    SE3 rel_matching = this_kf_matching_pose.inverse() * next_kf_matching_pose;
                    SE3 rel_gnss = this_kf_gps_pose.inverse() * next_kf_gps_pose;

                    // SE3 rel_matching = this_kf->matching_pose_.inverse() * next_kf->matching_pose_;
                    // SE3 rel_gnss = this_kf->gps_pose_.inverse() * next_kf->gps_pose_;

                    auto *edge = new core::CalibEdge(rel_gnss);
                    edge->setId(index);
                    edge->setVertex(0, vertex_pose);
                    edge->setInformation(Eigen::Matrix<double, 6, 6>::Identity());
                    edge->setMeasurement(rel_matching);

                    auto *rk = new g2o::RobustKernelCauchy;
                    rk->setDelta(calib_th);
                    edge->setRobustKernel(rk);

                    edges.push_back(edge);
                    optimizer.addEdge(edge);
                    index++;
                }
            }
        }
    }

    if (edges.size() < 10) {
        LOG(INFO) << "Calib edges less than 10, finish calib!";
        return;
    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(50);

    int calib_outlier_removed = 0;
    auto remove_outlier = [&calib_outlier_removed](g2o::OptimizableGraph::Edge *e) {
        // LOG(INFO) << "e->chi2(): "<<e->chi2()<<" "<<e->robustKernel()->delta();
        if (e->chi2() > e->robustKernel()->delta()) {
            e->setLevel(1);
            calib_outlier_removed++;
        } else {
            e->setRobustKernel(nullptr);
        }
    };
    std::for_each(edges.begin(), edges.end(), remove_outlier);

    LOG(INFO) << "Calib outliner/inliner: " << calib_outlier_removed << " " << edges.size() - calib_outlier_removed;
    if ((edges.size() - calib_outlier_removed) < 10) {
        LOG(INFO) << "Inlier calib edges less than 10, finish calib!";
        return;
    }

    impl_->optimizer_.setVerbose(false);
    impl_->optimizer_.initializeOptimization();
    impl_->optimizer_.optimize(10);

    impl_->TGL_ = SE3(vertex_pose->estimate().matrix());
    impl_->TGL_.translation()[2] = 0;  // 高度修不了

    boost::format fmt("重标定角度修正：%3.2f, %3.2f, %3.2f, X: %3.3f, Y: %3.3f");
    V3d ang = impl_->TGL_.so3().log() * 180 / M_PI;

    LogAndReport((fmt % ang[0] % ang[1] % ang[2] % impl_->TGL_.translation()[0] % impl_->TGL_.translation()[1]).str());
    LOG(INFO) << "TGL: \n" << impl_->TGL_.matrix();

    const double recalib_ang_th = 3.0;  // 重标定阈值
    const double recalib_x_th = 0.5;
    const double recalib_y_th = 0.5;
    if (impl_->TGL_.so3().log().norm() > recalib_ang_th * M_PI / 180 ||
        std::fabs(impl_->TGL_.translation()[0]) > recalib_x_th ||
        std::fabs(impl_->TGL_.translation()[1]) > recalib_y_th) {
        LogAndReport("重标定大于阈值：" + std::to_string(recalib_ang_th) + " " + std::to_string(recalib_x_th) + " " +
                     std::to_string(recalib_y_th) + ", 放弃重标定结果");
        return;
    } else {
        double ori_yaw = yaml_file_.GetValue<double>("velodyne_calib_param", "yaw");
        std::string lidar_type = yaml_file_.GetValue<std::string>("velodyne_calib_param", "factory");
        if (lidar_type == "suteng") {
            ori_yaw = ori_yaw + ang[2];
        } else if (lidar_type == "velodyne") {
            ori_yaw = ori_yaw - ang[2];
        }
        LogAndReport("重标定yaw角满足阈值， 重标定激光yaw角为: " + std::to_string(ori_yaw));
    }

    // 修正GNSS测量值
    std::map<IdType, common::KFPtr> keyframes_map;
    for (auto &kfp : impl_->keyframes_) {
        for (auto &this_kf : kfp.second) {
            keyframes_map.insert({this_kf->id_, this_kf});
        }
    }

    for (auto &gps_edge : impl_->gps_edges_) {
        if (gps_edge->level() != 0) {
            continue;
        }
        auto kfid = gps_edge->vertex(0)->id();
        gps_edge->setMeasurement(SE3ToG2OSE3Quat(keyframes_map[kfid]->gps_pose_ * impl_->TGL_));
    }
}

void OptimizationStage1::AddHeightFactors() {
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

    LOG(INFO) << "height factors: " << impl_->height_edges_.size();
}

void OptimizationStage1::MoveToOriginInGpsDenied() {
    if (impl_->optimization_params_.gps_status_set == 3 && impl_->if_use_nav_sat_) {
        /// GPS 良好
        LOG(INFO) << "skip because gps status is good.";
        return;
    }

    const double max_ave_trans = 1000.0;  // 认为不应该超过场地1km之外

    for (auto &kfp : impl_->keyframes_) {
        V3d trans = V3d::Zero();
        for (auto &kf : kfp.second) {
            trans += kf->optimized_pose_stage_1_.translation();
        }

        double ave_trans_norm = trans.norm() / trans.size();

        if (ave_trans_norm > max_ave_trans) {
            LogAndReport("发现弱GPS场地中轨迹" + std::to_string(kfp.first) + "平移过大：" +
                         std::to_string(ave_trans_norm) + "，平移此轨迹");
        }

        static V3d t_head = kfp.second[0]->optimized_pose_stage_1_.translation();
        for (auto &kf : kfp.second) {
            kf->optimized_pose_stage_1_.translation() -= t_head;
        }
    }
}

void OptimizationStage1::ConnectTrajectories() {
    /// 各轨迹的起始点和终点
    LOG(INFO) << "connecting trajectories";
    std::vector<IdType> start_points_mapping;
    std::vector<IdType> end_points_mapping;

    /// TODO 最好是只为弱GPS轨迹增加此段计算
    for (auto &tp : impl_->keyframes_) {
        if (tp.second[0]->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
            start_points_mapping.push_back(tp.second.front()->id_);
            end_points_mapping.push_back(tp.second.back()->id_);
        }
    }

    /// 在建图轨迹终点和起点处进行广域搜索
    const float grid_search_range_step = 1;
    const float grid_search_range = 3;
    const float grid_search_angle_step = 15;

    for (int i = 0; i < start_points_mapping.size() - 1; ++i) {
        auto e1 = end_points_mapping[i];
        auto s2 = start_points_mapping[i + 1];

        for (float dx = -grid_search_range; dx < grid_search_range; dx += grid_search_range_step) {
            for (float dy = -grid_search_range; dy < grid_search_range; dy += grid_search_range_step) {
                for (float angle = 0; angle < 360; angle += grid_search_angle_step) {
                    /// 搜索之
                    SE3 delta(AngAxisd(angle * M_PI / 180, V3d(0, 0, 1)).matrix(), V3d(dx, dy, 0));

                    auto kf_ref = impl_->keyframes_map_[e1];
                    auto kf = impl_->keyframes_map_[s2];

                    common::LoopCandidate c(kf->id_, kf_ref->id_);
                    c.Tij = delta;  // 假设位置
                    c.use_init_guess = true;
                    c.use_mm_match = true;
                    c.use_pose_stage = 1;

                    impl_->loop_candidates_.emplace_back(c);
                }
            }
        }
    }

    // go search
    core::MTSearchParams mt_params;
    mt_params.LoadFromYAML(yaml_file_);
    std::shared_ptr<core::MultiThreadSearch> mt_search;
    if (run_mode_ == RunMode::PIPELINE) {
        mt_search.reset(
            new core::MultiThreadSearch(local_data_path_, mt_params, impl_->keyframes_map_));  // 多线程搜索器
    } else if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        mt_search.reset(new core::MultiThreadSearch(local_db_path_, mt_params, impl_->keyframes_map_));  // 多线程搜索器
    }

    LOG(INFO) << "searching candidates: " << impl_->loop_candidates_.size();
    mt_search->ComputeConstraint(impl_->loop_candidates_);
    impl_->loop_candidates_ = mt_search->GetResults();

    if (!impl_->loop_candidates_.empty()) {
        AddLoopFactors();
    }
}

void OptimizationStage1::AddLoopFactors() {
    const double loop_noise_pos = 0.2;
    const double loop_noise_ang = 2.5 / 180.0 * M_PI;
    const double loop_th = impl_->optimization_params_.loop_chi2_th;

    for (auto &lc : impl_->loop_candidates_) {
        auto kf0 = impl_->keyframes_map_.at(lc.kfid_first);
        auto kf1 = impl_->keyframes_map_.at(lc.kfid_second);

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

        impl_->optimizer_.addEdge(e);
        impl_->loop_edges_.push_back(e);
    }

    LOG(INFO) << "loop factors: " << impl_->loop_edges_.size();
}

}  // namespace mapping::pipeline
