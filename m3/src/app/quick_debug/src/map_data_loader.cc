//
// Created by gaoxiang on 2021/2/18.
//

#include "map_data_loader.h"

#include <g2o/core/sparse_optimizer.h>
#include <glog/logging.h>

#include "common/mapping_math.h"
#include "common/num_type.h"
#include "common/std_headers.h"
#include "io/file_io.h"
#include "renderGL/mc_render_pass.h"
#include "renderGL/mc_technique_manager.h"

using namespace mapping;
using namespace HAMO;

const std::map<MapDataLoader::COLOR, HAMO::V4f> MapDataLoader::COLOR_MAP_{
    {MapDataLoader::COLOR::WHITE, {255.0, 255.0, 255.0, 1.0}},
    {MapDataLoader::COLOR::RED, {205.0, 0, 0, 1.0}},
    {MapDataLoader::COLOR::YELLOW, {255.0, 255.0, 0, 1.0}},
    {MapDataLoader::COLOR::GREEN, {0, 255.0, 0, 1.0}},
    {MapDataLoader::COLOR::BLUE, {0, 0, 255.0, 1.0}}};

MapDataLoader::MapDataLoader() {
    loading_ = false;
    running_ = false;
    std::atomic_init(&show_global_cloud_, true);
    std::atomic_init(&show_loop_frames_, true);
    std::atomic_init(&first_point_set_, false);
    std::atomic_init(&second_point_set_, false);
    std::atomic_init(&show_single_track_, false);
    std::atomic_init(&read_pose_from_db_, false);
    std::atomic_init(&fix_gnss_fixed_solution_, false);

    voxel_filter_.setLeafSize(0.08, 0.08, 0.08);

    graph_.reset(new g2o::SparseOptimizer());
}

MapDataLoader::~MapDataLoader() { Stop(); }

bool MapDataLoader::LoadDB(const std::string& directory) {
    db_path_ = directory + "/map.db";
    keyframe_txt_path_ = directory + "/keyframes.txt";
    hyper_graph_path_ = directory + "/s2_final.g2o";
    std::string loops_path = directory + "/loops.txt";
    std::string gps_path = directory + "/gps_path.txt";

    if (running_) {
        Stop();
    }

    // 读索引文件
    std::map<IdType, common::KFPtr> all_kfs;
    if (!io::LoadKeyframes(keyframe_txt_path_, all_kfs)) {
        LOG(ERROR) << "Load keyframes failed.";
        return false;
    }

    // add only the mapping kfs
    static int num_points = 0;
    static int num_fix = 0;

    if (read_pose_from_db_) {
        std::shared_ptr<io::DB_IO> db_io = std::make_shared<io::DB_IO>(db_path_);
        if (!db_io->ReadAllUniqueIdAndPose(map_id_pose_)) {
            LOG(ERROR) << "Failed to ReadAllUniqueIdAndPose.";
            std::map<int, SE3>().swap(map_id_pose_);
            db_io = nullptr;
            return false;
        }
    }

    for (auto& kfp : all_kfs) {
        if (kfp.second->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
            ++num_points;
            if (read_pose_from_db_) {
                kfp.second->optimized_pose_stage_2_ = map_id_pose_[kfp.first];
            }
            keyframe_map_.insert({kfp.first, kfp.second});
            index_map_.insert({Pos3d(kfp.second->optimized_pose_stage_2_.translation().cast<float>()), kfp.first});
            if (fix_gnss_fixed_solution_) {
                if (kfp.second->gps_status_ == common::GpsStatusType::GNSS_FIXED_SOLUTION) {
                    ++num_fix;
                    fixed_kfs_vec_.emplace_back(kfp.first);
                }
            }
        }
    }

    LOG(INFO) << "固定点数/总点数" << num_fix << " / " << num_points;

    LOG(INFO) << "loaded keyframes: " << keyframe_map_.size();

    LOG(INFO) << "loading pose graph...";
    if (nullptr == graph_) {
        graph_.reset(new g2o::SparseOptimizer());
    }
    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph_.get());

    std::ifstream ifs(hyper_graph_path_);
    if (!graph->load(ifs, true)) {
        return false;
    }

    LOG(INFO) << "nodes  : " << graph->vertices().size();
    LOG(INFO) << "edges  : " << graph->edges().size();

    // load loop.txt
    io::LoadLoopCandidates(loops_path, lc_);

    // load gps_path.txt
    std::vector<io::GpsInfo> gps_vec;
    io::LoadGpsInfo(gps_path, gps_vec);
    for (const auto& i : gps_vec) {
        if (i.fixed_inlier == 1) {
            fixed_kfs_vec_.emplace_back(i.id);
        }
    }

    // 开启读缓存线程
    db_io_ = std::make_shared<io::DB_IO>(db_path_);
    running_ = true;
    update_needed_ = true;
    load_thread_ = std::thread([this]() { PointCloudLoadThread(); });

    return true;
}

void MapDataLoader::PointCloudLoadThread() {
    loading_ = true;

    while (running_) {
        if (update_needed_) {
            // 读取必要的point cloud
            LoadNecessaryPointCloud();
            update_needed_ = false;
        }

        usleep(10000);
    }
}

void MapDataLoader::Draw(HAMO::RenderInfo& render_info) {
    UL lock(render_mutex_);

    bool draw_cur_kf = false, draw_kf1 = false, draw_kf2 = false;
    if (current_kf_ != nullptr) {
        current_kf_->render_leaf_->Render(render_info, nullptr);
        draw_cur_kf = true;
    }
    if (show_loop_frames_) {
        if (kf1_ != nullptr) {
            //            LOG(INFO);
            kf1_->render_leaf_->Render(render_info, nullptr);
            draw_kf1 = true;
        }

        if (kf2_ != nullptr) {
            kf2_->render_leaf_->Render(render_info, nullptr);
            draw_kf2 = true;
        }
    }

    auto should_skip = [](bool has_drawn, mapping::common::KFPtr kf, std::shared_ptr<LoadData> kf_to_check) -> bool {
        return (has_drawn) && (kf == kf_to_check->keyframe_ptr_);
    };

    if (show_global_cloud_) {
        for (auto iter = loaded_kfs_.begin(); iter != loaded_kfs_.end();) {
            if (iter->second->enable) {
                auto kf = iter->second->keyframe_ptr_;

                if (should_skip(draw_cur_kf, kf, current_kf_) || should_skip(draw_kf1, kf, kf1_) ||
                    should_skip(draw_kf2, kf, kf2_)) {
                    // 如果在currentkf, kf1, kf2中已经渲染，那么此处就跳过
                } else {
                    // 渲染
                    iter->second->render_leaf_->Render(render_info, nullptr);
                }

                iter++;
            } else {
                iter->second->Clean();
                iter = loaded_kfs_.erase(iter);
            }
        }
    }
}

void MapDataLoader::Update(const HAMO::Matrix4x4f& svMatrix, std::shared_ptr<HAMO::Camera> pCamera) {
    RenderTechnique* pTechnique = HAMO::TechniqueManager::GetInstance()->GetTechnique(0);
    RenderPass* pRenderPass = pTechnique->GetPass(0);
    Program* pProgram = pRenderPass->GetShaderProgram();
    UniformParam* param = pProgram->GetUniformParam();

    // 这个其实可以不用
    param->SetParameter(2, HAMO::V3f(-150, 150, 0.0f));  // 最小，最大高度
    param->SetParameter(3, HAMO::V3f(0, 255, 0.0f));     // 最小，最大intensity
    Matrix4x4f prjMatrix = pCamera->GetProjectionMatrix();

    M4f Tvs = svMatrix.ToEigen();

    UL lock(render_mutex_);
    auto update_kf_matrix = [&](std::shared_ptr<LoadData> ld) {
        M4f Tsm = ld->keyframe_ptr_->optimized_pose_stage_2_.matrix().cast<float>();
        Matrix4x4f mvMatrix(Tvs * Tsm);

        ld->render_leaf_->SetRenderTechnique(pTechnique);
        ld->render_leaf_->SetModelViewMatrix(mvMatrix);
        ld->render_leaf_->SetProjectionMatrix(prjMatrix);
    };

    for (auto& lkp : loaded_kfs_) {
        if (lkp.second != nullptr && lkp.second->keyframe_ptr_ != nullptr) {
            update_kf_matrix(lkp.second);
        }
    }

    if (current_kf_ != nullptr && current_kf_->keyframe_ptr_ != nullptr) {
        update_kf_matrix(current_kf_);
    }

    if (kf1_ != nullptr && kf1_->keyframe_ptr_ != nullptr) {
        update_kf_matrix(kf1_);
    }

    if (kf2_ != nullptr && kf2_->keyframe_ptr_ != nullptr) {
        update_kf_matrix(kf2_);
    }
}

void MapDataLoader::SetViewCenter(double x, double y) {
    if ((view_center_ - ::V2d(x, y)).norm() > 0.01) {
        update_needed_ = true;
    }

    view_center_ = ::V2d(x, y);
}

void MapDataLoader::LoadNecessaryPointCloud() {
    if (keyframe_map_.empty()) {
        return;
    }

    // LOG(INFO) << "loading necessary cloud";
    int num_loaded = 0, num_unloaded = 0;

    for (auto& kfp : keyframe_map_) {
        render_mutex_.lock();
        bool not_loaded = loaded_kfs_.find(kfp.first) == loaded_kfs_.end();
        render_mutex_.unlock();

        double distance = (kfp.second->optimized_pose_stage_2_.translation().head<2>() - view_center_).norm();

        if (!not_loaded && distance > load_distance_far_) {
            // 已经加载，但是出区
            UnLoadSingleKF(kfp.second);
            num_unloaded++;
        } else if (distance > load_distance_near_) {
            // 不需要加载
            continue;
        } else if (not_loaded) {
            // 加载点云并放入渲染部分
            auto ld = LoadSingleKF(kfp.second);
            UL lock(render_mutex_);
            loaded_kfs_.insert({ld->keyframe_ptr_->id_, ld});
            ++num_loaded;
        }
    }
}

void MapDataLoader::Stop() {
    running_ = false;
    if (load_thread_.joinable()) {
        load_thread_.join();
    }

    keyframe_map_.clear();
    loaded_kfs_.clear();
}

std::shared_ptr<MapDataLoader::LoadData> MapDataLoader::LoadSingleKF(common::KFPtr kf, bool custom_color,
                                                                     HAMO::V4f color) {
    db_io_->ReadSingleKF(kf->id_, kf, false);

    if (kf->cloud_ == nullptr || kf->cloud_->empty()) {
        LOG(ERROR) << "failed to load cloud " << kf->id_;
        return nullptr;
    }

    voxel_filter_.setInputCloud(kf->cloud_);
    common::PointCloudPtr cloud_f(new common::PointCloudType);
    voxel_filter_.filter(*cloud_f);

    // create drawable and render leaf
    auto ld = std::make_shared<LoadData>(kf);

    for (auto& pt : cloud_f->points) {
        ::V3d pw = ::V3d(pt.x, pt.y, pt.z);

        if (custom_color) {
            ld->point_drawable_->add(pw[0], pw[1], pw[2], color);
        } else {
            double height = (kf->optimized_pose_stage_2_ * pw)[2];
            auto clr = GetIntensityColor(height * 255.0 / 20.0);
            // auto clr = GetIntensityColor(pt.intensity);
            ld->point_drawable_->add(pw[0], pw[1], pw[2],
                                     {clr.blueF(), clr.greenF(), clr.redF(), float(pt.intensity) / 255.0f * 0.5f});
        }
    }

    kf->cloud_ = nullptr;  // remove raw point cloud
    ld->render_leaf_->SetDrawable(ld->point_drawable_.get());

    return ld;
}

bool MapDataLoader::CreateCurrentKFFromLoaded(IdType id, const HAMO::V4f& color) {
    auto kf = loaded_kfs_.at(id);
    if (kf == nullptr) {
        return false;
    }

    auto point_array = kf->point_drawable_->GetPointArray();

    auto create_kf = [this, &point_array, &kf, &color]() -> std::shared_ptr<LoadData> {
        auto ld = std::make_shared<LoadData>(kf->keyframe_ptr_);

        for (auto& pt_color : point_array) {
            ld->point_drawable_->add(pt_color.point.x, pt_color.point.y, pt_color.point.z, color);
        }

        ld->render_leaf_->SetDrawable(ld->point_drawable_.get());
        return ld;
    };

    if (mark_kf1_) {
        kf1_ = create_kf();
        mark_kf1_ = false;
    } else if (mark_kf2_) {
        kf2_ = create_kf();
        mark_kf2_ = false;
    } else {
        current_kf_ = create_kf();
    }

    return true;
}

void MapDataLoader::UnLoadSingleKF(mapping::common::KFPtr kf) {
    UL lock(render_mutex_);
    loaded_kfs_.at(kf->id_)->enable = false;
}

void MapDataLoader::UpdateSingleKF(common::KFPtr kf) {
    //    LOG(INFO);
    //    db_io_->ReadSingleKF(kf->id_, kf, false);
    if (kf->cloud_ == nullptr || kf->cloud_->empty()) {
        LOG(ERROR) << "failed to load cloud " << kf->id_;
        return;
    }

    voxel_filter_.setInputCloud(kf->cloud_);

    SE3 Twb = kf->optimized_pose_stage_2_;
    common::PointCloudPtr cloud_f(new common::PointCloudType);
    voxel_filter_.filter(*cloud_f);

    // create drawable and render leaf
    auto ld = std::make_shared<LoadData>(kf);
    for (auto& pt : cloud_f->points) {
        ::V3d pw = Twb * ::V3d(pt.x, pt.y, pt.z);
        auto clr = GetIntensityColor(pt.intensity);

        ld->point_drawable_->add(pw[0], pw[1], pw[2],
                                 {clr.redF(), clr.greenF(), clr.blueF(), float(pt.intensity) / 255.0f * 0.5f});
    }

    kf->cloud_ = nullptr;
    ld->render_leaf_->SetDrawable(ld->point_drawable_.get());

    UL lock(render_mutex_);
    loaded_kfs_.insert({kf->id_, ld});
}

void MapDataLoader::UpdateCurrentCloud(IdType selected_index, SE3& coordinate, const HAMO::V4f& color) {
    UL lock(render_mutex_);
    bool select_valid = keyframe_map_.find(selected_index) != keyframe_map_.end();
    bool already_loaded = loaded_kfs_.find(selected_index) != loaded_kfs_.end();

    if (!select_valid) {
        LOG(INFO) << "current kf id invalid: " << selected_index;
        return;
    } else {
        coordinate = keyframe_map_.at(selected_index)->optimized_pose_stage_2_;
    }

    if (nullptr != current_kf_ && selected_index == current_kf_->keyframe_ptr_->id_) {
        // 未改变
        return;
    }

    if (already_loaded) {
        // set from already loaded cloud
        CreateCurrentKFFromLoaded(selected_index, color);
    } else {
        // load new
        LOG(INFO) << "load new kf: " << selected_index;
        if (mark_kf1_) {
            kf1_ = LoadSingleKF(keyframe_map_.at(selected_index), true, COLOR_MAP_.at(COLOR::BLUE));
        } else if (mark_kf2_) {
            kf2_ = LoadSingleKF(keyframe_map_.at(selected_index), true, COLOR_MAP_.at(COLOR::GREEN));
        } else {
            current_kf_ = LoadSingleKF(keyframe_map_.at(selected_index), true, COLOR_MAP_.at(COLOR::WHITE));
        }
    }
}

void MapDataLoader::FinetuneX(int selected_index, double arg) {
    auto iter = keyframe_map_.find(selected_index);
    if (iter == keyframe_map_.end()) {
        LOG(INFO) << "无法找到该关键帧";
        return;
    }

    auto& cur_kf = iter->second;
    double old_arg = cur_kf->optimized_pose_stage_2_.translation()[0];
    cur_kf->optimized_pose_stage_1_.translation()[0] = arg;
    cur_kf->optimized_pose_stage_2_.translation()[0] = arg;
    if (fabs(old_arg - arg) > 1e-2) {
        // LoadSingleKF(cur_kf, true, COLOR_MAP_[COLOR::WHITE]);
    }
}

void MapDataLoader::FinetuneY(int selected_index, double arg) {
    auto iter = keyframe_map_.find(selected_index);
    if (iter == keyframe_map_.end()) {
        LOG(INFO) << "无法找到该关键帧";
        return;
    }

    auto& cur_kf = iter->second;
    double old_arg = cur_kf->optimized_pose_stage_2_.translation()[1];
    cur_kf->optimized_pose_stage_1_.translation()[1] = arg;
    cur_kf->optimized_pose_stage_2_.translation()[1] = arg;
    if (fabs(old_arg - arg) > 1e-2) {
        // LoadSingleKF(cur_kf, true, COLOR_MAP_[COLOR::WHITE]);
    }
}

void MapDataLoader::FinetuneZ(int selected_index, double arg) {
    auto iter = keyframe_map_.find(selected_index);
    if (iter == keyframe_map_.end()) {
        LOG(INFO) << "无法找到该关键帧";
        return;
    }

    auto& cur_kf = iter->second;
    double old_arg = cur_kf->optimized_pose_stage_2_.translation()[2];
    cur_kf->optimized_pose_stage_1_.translation()[2] = arg;
    cur_kf->optimized_pose_stage_2_.translation()[2] = arg;
    if (fabs(old_arg - arg) > 1e-2) {
        // LoadSingleKF(cur_kf, true, COLOR_MAP_[COLOR::WHITE]);
    }
}

void MapDataLoader::FinetuneRoll(int selected_index, double arg) {
    auto iter = keyframe_map_.find(selected_index);
    if (iter == keyframe_map_.end()) {
        LOG(INFO) << "无法找到该关键帧";
        return;
    }
    arg *= M_PI / 180;

    auto& cur_kf = iter->second;

    auto rpy = common::SE3ToRollPitchYaw(cur_kf->optimized_pose_stage_2_);
    double old_arg = rpy.roll;

    rpy.roll = arg;
    cur_kf->optimized_pose_stage_1_ = common::XYZRPYToSE3(rpy);
    cur_kf->optimized_pose_stage_2_ = common::XYZRPYToSE3(rpy);

    if (fabs(old_arg - arg) > 1e-3) {
        // LoadSingleKF(cur_kf, true, COLOR_MAP_[COLOR::WHITE]);
    }
}

void MapDataLoader::FinetunePitch(int selected_index, double arg) {
    auto iter = keyframe_map_.find(selected_index);
    if (iter == keyframe_map_.end()) {
        LOG(INFO) << "无法找到该关键帧";
        return;
    }
    arg *= M_PI / 180;

    auto& cur_kf = iter->second;

    auto rpy = common::SE3ToRollPitchYaw(cur_kf->optimized_pose_stage_2_);
    double old_arg = rpy.pitch;
    rpy.pitch = arg;
    cur_kf->optimized_pose_stage_1_ = common::XYZRPYToSE3(rpy);
    cur_kf->optimized_pose_stage_2_ = common::XYZRPYToSE3(rpy);

    if (fabs(old_arg - arg) > 1e-3) {
        // LoadSingleKF(cur_kf, true, COLOR_MAP_[COLOR_WHITE]);
    }
}

void MapDataLoader::FinetuneYaw(int selected_index, double arg) {
    auto iter = keyframe_map_.find(selected_index);
    if (iter == keyframe_map_.end()) {
        LOG(INFO) << "无法找到该关键帧";
        return;
    }
    arg *= M_PI / 180;

    auto& cur_kf = iter->second;

    auto rpy = common::SE3ToRollPitchYaw(cur_kf->optimized_pose_stage_2_);
    double old_arg = rpy.yaw;
    rpy.yaw = arg;
    cur_kf->optimized_pose_stage_1_ = common::XYZRPYToSE3(rpy);
    cur_kf->optimized_pose_stage_2_ = common::XYZRPYToSE3(rpy);

    if (fabs(old_arg - arg) > 1e-3) {
        // LoadSingleKF(cur_kf, true, COLOR_MAP_[COLOR_WHITE]);
    }
}

int MapDataLoader::AddLoop(const int id1, const int id2, const std::string yaml_file, Dialog::LoopCloseParams& params) {
    // 检测loop candidates之间的回环
    if (keyframe_map_.empty()) {
        return 0;
    }

    // go search
    core::MTSearchParams mt_params;
    mt_params.LoadFromYAML(yaml_file);

    std::shared_ptr<core::MultiThreadSearch> mt_search;

    io::YAML_IO yaml(yaml_file);
    std::string local_data_path = yaml.GetValue<std::string>("data_fetching", "local_data_path");

    mt_params.ndt_matching_min_proba = params.fit_thresh;
    mt_params.transformation_epsilon = params.trans_eps;
    mt_params.lidar_max_range = params.dis_thresh;
    mt_params.registration_method = params.method;
    mt_params.max_iterations = params.max_iter;
    mt_params.fitness_score_thresh = params.fit_thresh;

    mt_search.reset(new core::MultiThreadSearch(local_data_path, mt_params, keyframe_map_));  // 多线程搜索器

    DetectNodesConnectivityCandidate(id1, id2, params.method);

    LOG(INFO) << "searching candidates: " << loop_candidates_.size();

    mt_search->ComputeConstraint(loop_candidates_);

    loop_candidates_ = mt_search->GetResults();

    mapping::common::LoopCandidate candidate = loop_candidates_[0];

    io::SaveLoopCandidates(local_data_path + "loops.txt", loop_candidates_, true);

    if (loop_candidates_.size() > 0) {
        rebuild_optimization_ = true;
    }

    for (const auto& edge : loop_candidates_) {
        AddPair(edge.kfid_first, edge.kfid_second);
    }

    return loop_candidates_.size();
}

void MapDataLoader::DetectNodesConnectivityCandidate(const int n1, const int n2, const std::string method) {
    loop_candidates_.clear();
    if (keyframe_map_.empty()) {
        LOG(INFO);
        return;
    }

    int num = 10;

    for (int j = -num; j <= num; ++j) {
        int kb = n2 + j;
        if (kb >= keyframe_map_.size() || kb < 0) {
            continue;
        }
        for (int i = -num; i <= num; ++i) {
            int ka = n1 + i;
            if (ka >= keyframe_map_.size() || ka < 0) {
                continue;
            }
            common::LoopCandidate c(ka, kb);
            c.use_pose_stage = 2;
            if (method == "MM") {
                c.use_mm_match = true;
            } else {
                c.use_mm_match = false;
            }
            loop_candidates_.emplace_back(c);
        }
    }
    LOG(INFO) << "loop_candidates_.size() : " << loop_candidates_.size();
}

SE3 MapDataLoader::UpdateLoop(const int index, COLOR color) {
    SE3 coordinate;

    if (keyframe_map_.empty() || index < 0 || index >= keyframe_map_.size()) {
        LOG(INFO) << "current kf id invalid";
        return coordinate;
    }

    UpdateCurrentCloud(index, coordinate, COLOR_MAP_.at(color));

    return coordinate;
}

void MapDataLoader::FixCurrent(int id) {
    LOG(INFO) << "FIX keyframe " << id << " pose to " << std::endl
              << keyframe_map_.at(id)->optimized_pose_stage_2_.translation();

    rebuild_optimization_ = true;
}

void MapDataLoader::ResetLeafSize(const double res) { voxel_filter_.setLeafSize(res, res, res); }

bool MapDataLoader::PoseOptimization(const std::string yaml_path, const std::vector<int>& fixed_kf_ids,
                                     std::string& report, const OptimizationDialog::OptimizationParams& params) {
    mapping::io::YAML_IO yaml(yaml_path);
    if (yaml.IsOpened() == false) {
        LOG(ERROR) << "failed to open yaml";
        return false;
    }

    if (keyframe_map_.size() < 5) {
        LOG(ERROR) << "key frames not enough: " << keyframe_map_.size();
        return false;
    }

    for (const auto& i : keyframe_map_) {
        if (fixed_kfs_[i.second->trajectory_id_]) {
            fixed_kfs_vec_.emplace_back(i.first);
        }
    }
    if (!fixed_kf_ids.empty()) {
        for (const auto& i : fixed_kf_ids) {
            if (std::find(fixed_kfs_vec_.begin(), fixed_kfs_vec_.end(), i) == fixed_kfs_vec_.end()) {
                fixed_kfs_vec_.emplace_back(i);
            }
        }
    }

    mapping::pipeline::OptimizationStage2 opti_s2(yaml);

    if (!fixed_kfs_vec_.empty()) {
        opti_s2.LoadFixedKeyframesIndexes(fixed_kfs_vec_);
    }

    opti_s2.LoadKeyframesFromQuickDebug(keyframe_map_);
    if (opti_s2.Init() == false) {
        LOG(ERROR) << "failed to init opti s2";
        return false;
    }

    pipeline::OptimizationParams optimization_params;
    optimization_params.LoadFromYAML(yaml_path);
    optimization_params.use_rk_dr = false;
    optimization_params.use_rk_matching = false;
    if (rebuild_optimization_) {
        optimization_params.lidar_continous_num = 1;
        optimization_params.dr_continous_num = 2;
    }
    optimization_params.with_height = false;

    opti_s2.SetOptimizationParams(optimization_params);
    opti_s2.SetNotSavePcd();
    opti_s2.SetNotSaveDb();
    opti_s2.SetIteration(params.iteration_num);

    if (opti_s2.Start() == false) {
        LOG(ERROR) << "failed at opti s2";
        return false;
    }

    opti_s2.GenerateReport(report);

    if (rebuild_optimization_) {
        report.clear();

        optimizer_.reset(new mapping::pipeline::OptimizationStage2(yaml));

        if (optimizer_->Init() == false) {
            LOG(ERROR) << "failed to init opti s2";
            return false;
        }

        if (!fixed_kfs_vec_.empty()) {
            optimizer_->LoadFixedKeyframesIndexes(fixed_kfs_vec_);
        }

        pipeline::OptimizationParams optimization_params;
        optimization_params.LoadFromYAML(yaml_path);
        optimization_params.lidar_continous_num = 5;
        optimization_params.dr_continous_num = 2;
        optimization_params.with_height = params.with_height;
        optimization_params.with_height = false;
        optimizer_->SetOptimizationParams(optimization_params);

        optimizer_->SetNotSavePcd();
        optimizer_->SetIteration(params.iteration_num);
        optimizer_->SetNotSaveDb();

        if (optimizer_->Start() == false) {
            LOG(ERROR) << "failed at opti s2";
            return false;
        }
    }

    LOG(INFO) << "finish optimization";
    rebuild_optimization_ = false;

    UpdateLoadedKfs();

    return true;
}

void MapDataLoader::RebuildDisplayCloud() {
    if (running_) {
        std::map<IdType, std::shared_ptr<LoadData>>().swap(loaded_kfs_);
    }
    LoadNecessaryPointCloud();
}

bool MapDataLoader::UpdateLoadedKfs() {
    // 读索引文件
    std::map<IdType, common::KFPtr> all_kfs;
    if (!io::LoadKeyframes(keyframe_txt_path_, all_kfs)) {
        LOG(ERROR) << "Load keyframes failed.";
        return false;
    }

    IdType num = 0;

    std::map<Pos3d, IdType>().swap(index_map_);

    // add only the mapping kfs
    for (auto& kfp : all_kfs) {
        if (kfp.second->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
            if (loaded_kfs_.find(kfp.first) != loaded_kfs_.end()) {
                loaded_kfs_[kfp.first]->keyframe_ptr_->optimized_pose_stage_2_ = kfp.second->optimized_pose_stage_2_;
            } else {
                auto kf = std::make_shared<LoadData>(kfp.second);
                loaded_kfs_.insert({kfp.first, kf});
            }
            index_map_.insert({Pos3d(kfp.second->optimized_pose_stage_2_.translation().cast<float>()), kfp.first});

            ++num;
        }
    }

    LOG(INFO) << "changed keyframes: " << num;

    std::map<IdType, mapping::common::KFPtr>().swap(keyframe_map_);

    // add only the mapping kfs
    for (auto& kfp : all_kfs) {
        if (kfp.second->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
            keyframe_map_.insert({kfp.first, kfp.second});
        }
    }

    LOG(INFO) << "loaded keyframes: " << keyframe_map_.size();

    LOG(INFO) << "Update pose graph...";
    graph_->clear();
    g2o::SparseOptimizer* graph = dynamic_cast<g2o::SparseOptimizer*>(this->graph_.get());

    std::ifstream ifs(hyper_graph_path_);
    if (!graph->load(ifs, true)) {
        return false;
    }

    LOG(INFO) << "nodes  : " << graph->vertices().size();
    LOG(INFO) << "edges  : " << graph->edges().size();

    return true;
}

IdType MapDataLoader::GetSuggestKfIdx(IdType idx) {
    if (keyframe_map_.find(idx) != keyframe_map_.end()) {
        return idx;
    }

    auto iter =
        std::find_if(keyframe_map_.begin(), keyframe_map_.end(),
                     [&idx](const std::pair<const IdType, mapping::common::KFPtr>& ele) { return ele.first > idx; });

    if (iter == keyframe_map_.end()) {
        return 0;
    } else {
        LOG(INFO) << "suggest: " << iter->first << ", query: " << idx;
        return iter->first;
    }
}

void MapDataLoader::CloseMap() {
    if (running_) {
        Stop();
    }

    std::map<IdType, std::shared_ptr<LoadData>>().swap(loaded_kfs_);
    std::shared_ptr<LoadData>().swap(current_kf_);
    std::shared_ptr<LoadData>().swap(kf1_);
    std::shared_ptr<LoadData>().swap(kf2_);
    std::map<IdType, mapping::common::KFPtr>().swap(keyframe_map_);
    std::shared_ptr<g2o::HyperGraph>().swap(graph_);

    running_ = false;
}

void MapDataLoader::SetRubberBandPoint(const Eigen::Vector3f& point) {
    if (false == first_point_set_) {
        first_point_ = point;
        first_point_set_ = true;
        second_point_set_ = false;
    } else if (false == second_point_set_) {
        second_point_ = point;
        second_point_set_ = true;
        first_point_set_ = false;
    }
}