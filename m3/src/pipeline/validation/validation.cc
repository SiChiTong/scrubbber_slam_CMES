//
// Created by gaoxiang on 2020/10/22.
//

#include "pipeline/validation/validation.h"
#include "core/optimization/edge_se3_height_prior.h"
#include "io/file_io.h"
#include "pipeline/validation/validation_impl.h"
#include "src/common/candidate.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#include <glog/logging.h>
#include <boost/format.hpp>
#include <numeric>

namespace mapping::pipeline {

using namespace mapping::common;
using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<6, 6>>;
using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;

inline g2o::SE3Quat SE3ToG2OSE3Quat(const SE3 &T) { return g2o::SE3Quat(T.so3().matrix(), T.translation()); }

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

Validation::Validation(const io::YAML_IO &yaml_file) : impl_(new ValidationImpl), yaml_file_(yaml_file) {
    context_name_ = "Validation";
}

Validation::~Validation() noexcept {}

bool Validation::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }
    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");
    if (run_mode_ == PipelineContext::RunMode::GEM_EXECUTABLE) {
        local_db_path_ = yaml_file_.GetValue<std::string>("local_db_path");
    }
    impl_->params_.LoadFromYAML(yaml_file_);
    impl_->optimization_params_.LoadFromYAML(yaml_file_);
    impl_->gps_status_ = yaml_file_.GetValue<int>("gps_status");

    // load car type
    std::string car_type = yaml_file_.GetValue<std::string>("car_type");
    if (car_type == "wxx") {
        impl_->car_type_ = common::CarType::WXX;
    } else if (car_type == "lads") {
        impl_->car_type_ = common::CarType::LADS;
    } else {
        impl_->car_type_ = common::CarType::OTHERS;
        LOG(ERROR) << "Unknown car type: " << car_type;
        return false;
    }

    if (!io::LoadKeyframes(local_data_path_ + "keyframes.txt", impl_->keyframes_)) {
        LogAndReport("未能读取关键帧信息");
        return false;
    }

    int index = 0;
    for (auto &t : impl_->keyframes_) {
        for (const auto &kf : t.second) {
            impl_->keyframes_map_by_id_.insert({kf->id_, kf});

            if (kf->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
                /// mapping bags建立kd树
                impl_->keyframes_map_by_id_mapping_.insert({kf->id_, kf});
                V2d xy = kf->optimized_pose_stage_2_.translation().head<2>();
                pcl::PointXY pxy{};
                pxy.x = xy[0];
                pxy.y = xy[1];
                impl_->kfs_2d_cloud_->points.push_back(pxy);
                impl_->kfs_cloud_idx_to_id_.insert({index, kf->id_});
                index++;
            } else {
                // validation bags
                kf->optimized_pose_stage_2_ = kf->optimized_pose_stage_1_;  //  验证的keyframe没有第二阶段pose
                impl_->keyframes_map_by_id_validation_.insert({kf->id_, kf});
            }
        }
    }

    impl_->mt_search_ =
        std::make_shared<core::MultiThreadSearch>(local_data_path_, impl_->params_, impl_->keyframes_map_by_id_);
    return true;
}

bool Validation::Start() {
    LogAndReport("开始验证环节");
    SetStatus(ContextStatus::WORKING);

    if (impl_->car_type_ == common::CarType::LADS) {
        LogAndReport("LADS车辆没有验证环节");
        SetStatus(ContextStatus::SUCCEED);
        return true;
    }

    BuildProblem();

    std::vector<int> success_per_traj = CheckKfsWithGoodGPS();
    Solve();
    impl_->optimizer_.save((local_data_path_ + "valid_s1.g2o").c_str());

    std::string dr_repo = std::string("DR ") + print_info(impl_->dr_edges_, impl_->optimization_params_.dr_chi2_th);
    std::string loop_repo =
        std::string("Loop closure ") + print_info(impl_->loop_edges_, impl_->optimization_params_.loop_chi2_th);
    LogAndReport(dr_repo);
    LogAndReport(loop_repo);

    float checked_ratio = float(impl_->kf_localized_map_.size()) / impl_->keyframes_map_by_id_validation_.size();
    LogAndReport("GNSS验证比例：" + std::to_string(checked_ratio));

    const float gps_check_ratio_th = 0.05;

    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
            continue;
        }

        float checked_ratio = success_per_traj[kfp.first] / float(kfp.second.size());
        if (checked_ratio > gps_check_ratio_th) {
            CheckWithSmoothingResults(kfp.first);
            Solve();
            checked_ratio = impl_->kf_localized_map_.size() / float(impl_->keyframes_map_by_id_validation_.size());
            LogAndReport("轨迹 " + std::to_string(kfp.first) + " GNSS平滑检查比例: " + std::to_string(checked_ratio));
        } else {
            LogAndReport("轨迹 " + std::to_string(kfp.first) + " 使用广域搜索确定起点或终点");
            int num_checked_es = CheckWithExhaustiveSearch(kfp.first);
            //impl_->optimizer_.save((local_data_path_ + "valid_ex_search_before.g2o").c_str());
            Solve();
            impl_->optimizer_.save((local_data_path_ + "valid_ex_search.g2o").c_str());
            LogAndReport("广域搜索匹配数：" + std::to_string(num_checked_es));

            if (num_checked_es > 0) {
                // 广域搜索有效
                AddHeightFactors();
                CheckWithSmoothingResults(kfp.first);
                Solve();
            }
        }
    }

    checked_ratio = impl_->kf_localized_map_.size() / float(impl_->keyframes_map_by_id_validation_.size());
    float last_checked_ratio = checked_ratio;

    const float check_ratio_th = 0.972;
    const float check_ratio_inc_th = 1e-3;
    int num_iters = 0;

    while (num_iters < 1000) {
        num_iters++;
        if (checked_ratio < check_ratio_th) {
            // 继续检查
            LOG(INFO) << "checked ratio not enough: " << checked_ratio;
            CheckWithSmoothingResults(search_all_);
            checked_ratio = impl_->kf_localized_map_.size() / float(impl_->keyframes_map_by_id_validation_.size());
            Solve();
            impl_->optimizer_.save((local_data_path_ + "valid_iter_" + std::to_string(num_iters) + ".g2o").c_str());
            LogAndReport("检测轮数: " + std::to_string(num_iters) + ", 检查比例: " + std::to_string(checked_ratio));

            if (checked_ratio < last_checked_ratio + check_ratio_inc_th) {
                LogAndReport("无法找到更多的连接，中止匹配: " + std::to_string(checked_ratio) + "<" +
                             std::to_string(last_checked_ratio + check_ratio_inc_th));
                break;
            }

        } else {
            LOG(INFO) << "checked ratio enough: " << checked_ratio;
            break;
        }

        last_checked_ratio = checked_ratio;
    }

    LogAndReport("迭代次数：" + std::to_string(num_iters));
    impl_->optimizer_.save((local_data_path_ + "valid_final.g2o").c_str());

    dr_repo = std::string("DR ") + print_info(impl_->dr_edges_, impl_->optimization_params_.dr_chi2_th);
    dr_repo = std::string("Heights ") + print_info(impl_->height_edges_, 0);
    loop_repo = std::string("Loop closure ") + print_info(impl_->loop_edges_, impl_->optimization_params_.loop_chi2_th);
    LogAndReport(dr_repo);
    LogAndReport(loop_repo);

    if (checked_ratio < check_ratio_th) {
        LogAndReport("\\textbf{警告}：无法找到足够的定位匹配，比例：" + std::to_string(checked_ratio) +
                     "，部分贴边轨迹可能不准确");
    }

    SaveResults();

    LogAndReport("验证环节结束");
    SetStatus(ContextStatus::SUCCEED);
    return true;
}

void Validation::SaveResults() {
    io::SaveKeyframeSinglePath(local_data_path_ + "optimization2.txt", impl_->keyframes_,
                               io::SaveKeyframePathType::OPTI_PATH_STAGE_2);
    io::RemoveIfExist(local_data_path_ + "keyframes.txt");
    io::SaveKeyframePose(local_data_path_ + "keyframes.txt", impl_->keyframes_);

    io::SaveLoopCandidatesScore(local_data_path_ + "loops_score.txt", impl_->loops_score_result_,
                                impl_->keyframes_map_by_id_validation_, true);

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        SaveGemReport(local_data_path_ + gem_report_name_);
    }

    LogAndReport("验证结果已保存");
}

int Validation::CheckWithSmoothingResults(int traj_id) {
    int num_checked = 0;

    impl_->loops_score_result_.insert(impl_->loops_score_result_.end(),
                                      impl_->loop_candidates_smoothing_search_result_.begin(),
                                      impl_->loop_candidates_smoothing_search_result_.end());

    impl_->loop_candidates_smoothing_search_.clear();
    impl_->loop_candidates_smoothing_search_result_.clear();

    std::map<IdType, common::KFPtr> kfs_valid;
    if (traj_id == -1) {
        kfs_valid = impl_->keyframes_map_by_id_validation_;
    } else {
        assert(impl_->keyframes_[traj_id].front()->bag_type_ == KeyFrameBagType::VALIDATION_BAGS);
        for (auto &kf : impl_->keyframes_[traj_id]) {
            kfs_valid.insert({kf->id_, kf});
        }
    }

    for (auto &kfp : kfs_valid) {
        auto kf = kfp.second;

        if (kf->gps_inlier_ && common::GpsUsable(kf->gps_status_)) {
            continue;
        }

        if (impl_->kf_localized_map_.find(kf) != impl_->kf_localized_map_.end()) {
            /// 已经检查过
            continue;
        }

        /// 找最近的
        std::vector<int> indicies;
        std::vector<float> distances;
        pcl::PointXY pt_xy{};
        pt_xy.x = kf->optimized_pose_stage_2_.translation()[0];
        pt_xy.y = kf->optimized_pose_stage_2_.translation()[1];

        impl_->kdtree_->nearestKSearch(pt_xy, 40, indicies, distances);
        if (indicies.empty()) {
            continue;
        }

        // 选取一部分我们想检测回环的
        std::vector<int> indicies_take;
        for (int &idx : indicies) {
            if (std::find_if(indicies_take.begin(), indicies_take.end(),
                             [&idx](int id) { return abs(id - idx) < 20; }) == indicies_take.end()) {
                indicies_take.emplace_back(idx);
            }

            if (indicies_take.size() > 2) break;
        }

        for (auto &idx : indicies_take) {
            auto kf_ref = impl_->keyframes_map_by_id_mapping_[impl_->kfs_cloud_idx_to_id_[idx]];

            // create candidate
            common::LoopCandidate c(kf->id_, kf_ref->id_);
            c.Tij = kf->optimized_pose_stage_2_.inverse() * kf_ref->optimized_pose_stage_2_;
            c.use_mm_match = true;
            c.use_pose_stage = 2;
            impl_->loop_candidates_smoothing_search_.emplace_back(c);
        }
    }

    impl_->mt_search_->ComputeConstraint(impl_->loop_candidates_smoothing_search_);
    impl_->loop_candidates_smoothing_search_result_ = impl_->mt_search_->GetResults();

    for (auto &res : impl_->loop_candidates_smoothing_search_result_) {
        auto kf = impl_->keyframes_map_by_id_[res.kfid_first];
        if (impl_->kf_localized_map_.find(kf) == impl_->kf_localized_map_.end()) {
            impl_->kf_localized_map_.insert({kf, 1});
        } else {
            impl_->kf_localized_map_[kf]++;
        }
    }

    num_checked = impl_->loop_candidates_smoothing_search_result_.size();
    LOG(INFO) << "根据平滑结果添加的定位数量：" << num_checked << ", 总关键帧："
              << impl_->keyframes_map_by_id_validation_.size() << ", 检测过的：" << impl_->kf_localized_map_.size();

    if (traj_id != -1) {
        LOG(INFO) << "trajectory " << traj_id << " checked ratio: " << float(num_checked) / kfs_valid.size();
    }

    // add factors
    const double loop_noise_pos = 0.2;
    const double loop_noise_ang = 2.5 / 180.0 * M_PI;
    const double loop_th = impl_->optimization_params_.loop_chi2_th;

    for (auto &lc : impl_->loop_candidates_smoothing_search_result_) {
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
    return num_checked;
}

int Validation::CheckWithExhaustiveSearch(int traj_id) {
    LogAndReport("使用广域搜索");
    int num_checked = 0;

    auto &kfs_valid = impl_->keyframes_[traj_id];

    /// 各轨迹的起始点和终点
    std::vector<IdType> endpoints_mapping;
    std::vector<IdType> endpoints_validation;

    for (auto &tp : impl_->keyframes_) {
        if (tp.second[0]->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
            endpoints_mapping.push_back(tp.second.front()->id_);
            endpoints_mapping.push_back(tp.second.back()->id_);
        }
    }

    endpoints_validation.push_back(kfs_valid.front()->id_);
    endpoints_validation.push_back(kfs_valid.back()->id_);

    /// 在建图轨迹终点和起点处进行广域搜索
    const float grid_search_range_step = 2;
    const float grid_search_range = 10;
    const float grid_search_angle_step = 15;

    /// 禁 忌 的 五 重 循 环
    for (auto &epv : endpoints_validation) {
        for (auto &epm : endpoints_mapping) {
            for (float dx = -grid_search_range; dx < grid_search_range; dx += grid_search_range_step) {
                for (float dy = -grid_search_range; dy < grid_search_range; dy += grid_search_range_step) {
                    for (float angle = 0; angle < 360; angle += grid_search_angle_step) {
                        /// 搜索之
                        SE3 delta(AngAxisd(angle * M_PI / 180, V3d(0, 0, 1)).matrix(), V3d(dx, dy, 0));

                        auto kf = impl_->keyframes_map_by_id_validation_[epv];
                        auto kf_ref = impl_->keyframes_map_by_id_mapping_[epm];

                        common::LoopCandidate c(kf->id_, kf_ref->id_);
                        c.Tij = delta;  // 假设位置
                        c.use_init_guess = true;
                        c.use_mm_match = false;
                        c.use_pose_stage = 2;
                        impl_->loop_candidates_exhaustive_search_.emplace_back(c);
                    }
                }
            }
        }
    }

    LOG(INFO) << "initial guess: " << impl_->loop_candidates_exhaustive_search_.size();
    impl_->mt_search_->ComputeConstraint(impl_->loop_candidates_exhaustive_search_);
    impl_->loop_candidates_exhaustive_search_result_ = impl_->mt_search_->GetResults();

    for (auto &res : impl_->loop_candidates_exhaustive_search_result_) {
        auto kf = impl_->keyframes_map_by_id_[res.kfid_first];
        if (impl_->kf_localized_map_.find(kf) == impl_->kf_localized_map_.end()) {
            impl_->kf_localized_map_.insert({kf, 1});
        } else {
            impl_->kf_localized_map_[kf]++;
        }
    }

    num_checked = impl_->loop_candidates_exhaustive_search_result_.size();
    LogAndReport("根据广域搜索添加的定位数量：" + std::to_string(num_checked));

    // 转换整条轨迹
    if (impl_->loop_candidates_exhaustive_search_result_.empty()) {
        return 0;
    }

    // 找个分值最高的
    auto &res = impl_->loop_candidates_exhaustive_search_result_;
    auto best_iter = std::max_element(
        res.begin(), res.end(), [](const LoopCandidate &r1, const LoopCandidate &r2) { return r1.score > r2.score; });

    auto best_result = *best_iter;

    SE3 Twr = impl_->keyframes_map_by_id_[best_result.kfid_second]->optimized_pose_stage_2_;
    SE3 Twc = impl_->keyframes_map_by_id_[best_result.kfid_first]->optimized_pose_stage_1_;

    //impl_->optimizer_.save((local_data_path_ + "valid_s1_trans_before.g2o").c_str());
    SE3 real_Twc = Twr * best_result.Tij.inverse();
    SE3 delta = real_Twc * Twc.inverse();
    LOG(INFO) << "resetting estimate value";
    for (auto &kf : kfs_valid) {
        impl_->kfid_to_vertices_[kf->id_]->setEstimate(SE3ToG2OSE3Quat(delta * kf->optimized_pose_stage_2_));
    }
    //impl_->optimizer_.save((local_data_path_ + "valid_s1_trans_after.g2o").c_str());

    // add factors
    const double loop_noise_pos = 0.2;
    const double loop_noise_ang = 2.5 / 180.0 * M_PI;
    const double loop_th = impl_->optimization_params_.loop_chi2_th;

    for (auto &lc : impl_->loop_candidates_exhaustive_search_result_) {
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

        auto *rk = new g2o::RobustKernelHuber;
        rk->setDelta(loop_th * impl_->loop_exhaustive_rk_factor_);
        e->setRobustKernel(rk);

        impl_->optimizer_.addEdge(e);
        impl_->loop_edges_.push_back(e);
    }

    impl_->loops_score_result_.insert(impl_->loops_score_result_.end(),
                                      impl_->loop_candidates_exhaustive_search_result_.begin(),
                                      impl_->loop_candidates_exhaustive_search_result_.end());

    LOG(INFO) << "loop factors: " << impl_->loop_edges_.size();
    return num_checked;
}

void Validation::BuildProblem() {
    auto *solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    auto *offset_value = new g2o::ParameterSE3Offset();
    offset_value->setId(0);
    impl_->optimizer_.addParameter(offset_value);
    impl_->optimizer_.setAlgorithm(solver);

    /// 固定建图的关键帧
    for (auto &kf : impl_->keyframes_map_by_id_mapping_) {
        auto *v = new g2o::VertexSE3();
        v->setEstimate(SE3ToG2OSE3Quat(kf.second->optimized_pose_stage_2_));
        v->setId(kf.second->id_);
        v->setFixed(true);
        impl_->optimizer_.addVertex(v);
        impl_->vertices_.emplace_back(v);
        impl_->kfid_to_vertices_.insert({kf.second->id_, v});
    }

    /// 验证的关键帧
    for (auto &kf : impl_->keyframes_map_by_id_validation_) {
        auto *v = new g2o::VertexSE3();
        v->setEstimate(SE3ToG2OSE3Quat(kf.second->optimized_pose_stage_2_));
        v->setId(kf.second->id_);
        impl_->optimizer_.addVertex(v);
        impl_->vertices_.emplace_back(v);
        impl_->kfid_to_vertices_.insert({kf.second->id_, v});
    }

    /// 验证的DR
    AddDRFactors();
    AddMatchingFactors();

    LOG(INFO) << "vertices: " << impl_->vertices_.size();
}

void Validation::AddMatchingFactors() {
    size_t num = impl_->optimization_params_.lidar_continous_num;
    double matching_th = impl_->optimization_params_.matching_chi2_th;

    LOG(INFO) << "adding matching";
    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
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

void Validation::AddDRFactors() {
    /// 只处理验证的轨迹
    size_t dr_continous_frames = impl_->optimization_params_.dr_continous_num;
    double dr_th = impl_->optimization_params_.dr_chi2_th;
    for (auto &kfp : impl_->keyframes_) {
        if (kfp.second[0]->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
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

                impl_->optimizer_.addEdge(e);
                impl_->dr_edges_.push_back(e);
            }
        }
    }
}

std::vector<int> Validation::CheckKfsWithGoodGPS() {
    LOG(INFO) << "check kfs with good gps";
    std::vector<int> checked_per_traj(impl_->keyframes_.size(), 0);
    auto &kfs_valid = impl_->keyframes_map_by_id_validation_;
    impl_->kdtree_->setInputCloud(impl_->kfs_2d_cloud_);

    for (auto &kfp : kfs_valid) {
        auto kf = kfp.second;

        if (!kf->gps_inlier_ || !common::GpsUsable(kf->gps_status_)) {
            continue;
        }

        /// 找最近的
        std::vector<int> indicies;
        std::vector<float> distances;
        pcl::PointXY pt_xy{};
        pt_xy.x = kf->optimized_pose_stage_1_.translation()[0];
        pt_xy.y = kf->optimized_pose_stage_1_.translation()[1];
        impl_->kdtree_->nearestKSearch(pt_xy, 40, indicies, distances);

        if (indicies.empty()) {
            LOG(INFO) << "indicies.empty()";
            continue;
        }

        // 选取一部分我们想检测回环的
        std::vector<int> indicies_take;
        for (int &idx : indicies) {
            if (std::find_if(indicies_take.begin(), indicies_take.end(),
                             [&idx](int id) { return abs(id - idx) < 20; }) == indicies_take.end()) {
                indicies_take.emplace_back(idx);
            }

            if (indicies_take.size() > 2) break;
        }

        for (auto &idx : indicies_take) {
            auto kf_ref = impl_->keyframes_map_by_id_mapping_[impl_->kfs_cloud_idx_to_id_[idx]];

            // create candidate
            common::LoopCandidate c(kf->id_, kf_ref->id_);
            c.Tij = kf->optimized_pose_stage_1_.inverse() * kf_ref->optimized_pose_stage_1_;
            c.use_mm_match = true;  // 由于GNSS高度可能存在问题，这里使用MM match消除高度问题
            c.use_pose_stage = 2;
            impl_->loop_candidates_gnss_search_.emplace_back(c);
        }
    }

    impl_->mt_search_->ComputeConstraint(impl_->loop_candidates_gnss_search_);
    impl_->loop_candidates_gnss_search_result_ = impl_->mt_search_->GetResults();

    /// 统计每条轨迹
    for (auto &res : impl_->loop_candidates_gnss_search_result_) {
        auto kf = impl_->keyframes_map_by_id_[res.kfid_first];
        if (impl_->kf_localized_map_.find(kf) == impl_->kf_localized_map_.end()) {
            impl_->kf_localized_map_.insert({kf, 1});
        } else {
            impl_->kf_localized_map_[kf]++;
        }

        checked_per_traj[kf->trajectory_id_]++;
    }

    // add factors
    const double loop_noise_pos = 0.2;
    const double loop_noise_ang = 2.5 / 180.0 * M_PI;
    const double loop_th = impl_->optimization_params_.loop_chi2_th;

    for (auto &lc : impl_->loop_candidates_gnss_search_result_) {
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

        auto *rk = new g2o::RobustKernelHuber;
        rk->setDelta(loop_th * impl_->loop_exhaustive_rk_factor_);
        e->setRobustKernel(rk);

        impl_->optimizer_.addEdge(e);
        impl_->loop_edges_.push_back(e);
    }
    LOG(INFO) << "loop factors: " << impl_->loop_edges_.size();

    impl_->loops_score_result_.insert(impl_->loops_score_result_.end(),
                                      impl_->loop_candidates_gnss_search_result_.begin(),
                                      impl_->loop_candidates_gnss_search_result_.end());

    return checked_per_traj;
}

void Validation::Solve() {
    impl_->optimizer_.setVerbose(false);
    impl_->optimizer_.initializeOptimization();
    impl_->optimizer_.optimize(200);

    // fetch results
    for (auto *v : impl_->vertices_) {
        impl_->keyframes_map_by_id_[v->id()]->optimized_pose_stage_2_ = SE3(v->estimate().matrix());
    }

    // update average height
    // if (!impl_->height_edges_.empty()) {
    //     std::vector<double> ave_height_global;  // 全局高度值
    //     for (auto &v : impl_->vertices_) {
    //         ave_height_global.push_back(v->estimate().translation()[2]);
    //     }

    //     double ave_height = ave_height_global[int(ave_height_global.size() / 2)];
    //     LOG(INFO) << "ave height changed from " << impl_->average_height_ << " to " << ave_height;
    //     impl_->average_height_ = ave_height;

    //     for (auto &e : impl_->height_edges_) {
    //         e->setMeasurement(impl_->average_height_);
    //     }
    // }
}

void Validation::AddHeightFactors() {
    /// 用建图包的平均高度来确定高度值
    std::vector<double> ave_height_global;  // 全局高度值
    for (auto &kfp : impl_->keyframes_map_by_id_mapping_) {
        ave_height_global.push_back(kfp.second->optimized_pose_stage_2_.translation()[2]);
    }

    if (ave_height_global.empty()) {
        return;
    }

    const double height_noise = 0.1;
    impl_->average_height_ = ave_height_global[int(ave_height_global.size() / 2)];

    for (auto &tp : impl_->keyframes_) {
        if (tp.second[0]->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
            continue;
        }

        for (auto kf : tp.second) {
            int id = kf->id_;
            auto *e = new core::EdgeSE3HeightPrior();
            e->setId(impl_->edge_id_++);
            e->setVertex(0, impl_->optimizer_.vertex(id));

            M1d cov = M1d::Identity() * height_noise * height_noise;
            e->setInformation(cov.inverse());
            e->setMeasurement(impl_->average_height_);
            impl_->optimizer_.addEdge(e);
            impl_->height_edges_.push_back(e);
        }
    }
    LogAndReport("增加高度约束: 平均高度=" + std::to_string(impl_->average_height_));
}

}  // namespace mapping::pipeline
