//
// Created by gaoxiang on 2020/9/22.
//

#include "pipeline/loop_closing/loop_closing.h"
#include "io/file_io.h"
#include "pipeline/loop_closing/loop_closing_impl.h"

#include <glog/logging.h>

namespace mapping::pipeline {

using namespace mapping::common;

LoopClosing::LoopClosing(const io::YAML_IO &yaml_file, RunMode run_mode, bool only_detect)
    : yaml_file_(yaml_file), impl_(new LoopClosingImpl), PipelineContext(run_mode) {
    context_name_ = "LoopClosing";
    impl_->only_detect_ = only_detect;
}

LoopClosing::~LoopClosing() noexcept { LOG(INFO) << "Loop closing deconstructed."; }

void LoopClosing::SetStartEndIdx(IdType start_id, IdType end_id) {
    impl_->start_id_ = start_id;
    impl_->end_id_ = end_id;
}

bool LoopClosing::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }
    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");
    local_db_path_ = yaml_file_.GetValue<std::string>("local_db_path");
    if_merge_maps_ = yaml_file_.GetValue<bool>("if_merge_maps");
    impl_->params_.LoadFromYAML(yaml_file_);

    impl_->gps_status_ = yaml_file_.GetValue<int>("gps_status");

    std::string keyframes_txt_path;
    if (if_merge_maps_) {
        LogAndReport("加载新增地图合并的关键帧文件");
        keyframes_txt_path = local_data_path_ + "merge_keyframes.txt";
    } else {
        keyframes_txt_path = local_data_path_ + "keyframes.txt";
    }
    if (!io::LoadKeyframes(keyframes_txt_path, impl_->keyframes_)) {
        LogAndReport("无法读取关键帧文件");
        return false;
    }

    int index = 0;
    for (auto &kfp : impl_->keyframes_) {
        for (auto &kf : kfp.second) {
            impl_->keyframes_by_id_.insert({kf->id_, kf});
            if (kf->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
                impl_->keyframes_by_id_mapping_.insert({kf->id_, kf});
                V2d xy = kf->optimized_pose_stage_1_.translation().head<2>();
                pcl::PointXY pxy;
                pxy.x = xy[0];
                pxy.y = xy[1];

                impl_->kfs_2d_cloud_->points.push_back(pxy);
                impl_->kfs_cloud_idx_to_id_.insert({index, kf->id_});
                index++;
            } else {
                impl_->keyframes_by_id_validation_.insert({kf->id_, kf});
            }
        }
    }

    if (run_mode_ == PipelineContext::RunMode::GEM_EXECUTABLE) {
        /// GEM中由进程调用，因此线程数为1
        impl_->params_.num_threads = 1;
        if (!impl_->only_detect_) {
            // load loop.txt
            std::vector<LoopCandidate> lc;
            io::LoadLoopCandidates(local_data_path_ + "loops.txt", lc);
            // select the picked one
            impl_->loop_candidates_ = {lc.begin() + impl_->start_id_, lc.begin() + impl_->end_id_};
        }
    }

    if (!impl_->only_detect_) {
        /// 需要计算时，才构建mt search
        if (run_mode_ == PipelineContext::RunMode::PIPELINE) {
            impl_->mt_search_ = std::make_shared<core::MultiThreadSearch>(local_data_path_, impl_->params_,
                                                                          impl_->keyframes_by_id_, false);
        } else {
            impl_->mt_search_ = std::make_shared<core::MultiThreadSearch>(local_db_path_, impl_->params_,
                                                                          impl_->keyframes_by_id_, true);
        }
    }
    return true;
}

bool LoopClosing::Start() {
    SetStatus(ContextStatus::WORKING);

    /// 第一步，根据距离和夹角信息枚举可能存在的回环
    DetectLoopCandidates();

    /// 第二步，计算相对运动
    ComputeRelativeMotionMT();

    /// 第三步，存储回环信息
    SaveLoopCandidates();

    Clear();

    LogAndReport("回环检查结束");
    SetStatus(ContextStatus::SUCCEED);
    return true;
}

void LoopClosing::Clear() {
    impl_->mt_search_ = nullptr;
    impl_->keyframes_.clear();
    impl_->keyframes_by_id_.clear();
    impl_->loop_candidates_.clear();
}

void LoopClosing::DetectLoopCandidates() {
    DetectByStage1();
    // DetectByContinuous();

    LogAndReport("建图回环检查点：" + std::to_string(impl_->loop_candidates_.size()));
}

void LoopClosing::DetectByStage1() {
    KFPtr check_first = nullptr;
    KFPtr check_second = nullptr;

    auto &kfs_mapping = impl_->keyframes_by_id_mapping_;
    for (auto iter_first = kfs_mapping.begin(); iter_first != kfs_mapping.end(); ++iter_first) {
        auto kf_first = iter_first->second;

        if (kf_first->matching_eigen_value_ <= 150) {
            // 不要寻找退化的匹配
            continue;
        }

        if (check_first != nullptr && kf_first->trajectory_id_ == check_first->trajectory_id_ &&
            abs(int(kf_first->id_ - check_first->id_)) <= impl_->params_.min_id_interval_) {
            // 同条轨迹内，跳过一定的ID区间
            continue;
        }

        for (auto iter_second = iter_first; iter_second != kfs_mapping.end(); ++iter_second) {
            auto kf_second = iter_second->second;
            if (kf_second->matching_eigen_value_ <= 150) {
                // 不要寻找退化的匹配
                continue;
            }

            if (check_second != nullptr && kf_second->trajectory_id_ == check_second->trajectory_id_ &&
                abs(int(kf_second->id_ - check_second->id_)) <= impl_->params_.min_id_interval_) {
                continue;
            }
            if (kf_first->trajectory_id_ == kf_second->trajectory_id_ &&
                abs(int(kf_first->id_ - kf_second->id_)) < impl_->params_.closest_id_th_) {
                /// 在同一条轨迹中，如果间隔太近，就不考虑回环
                continue;
            }
            V3d dt = kf_first->optimized_pose_stage_1_.translation() - kf_second->optimized_pose_stage_1_.translation();
            double t2d = dt.head<2>().norm();  // x-y distance
            double range_th = impl_->params_.lidar_max_range;
            if (t2d < range_th) {
                LoopCandidate c(kf_first->id_, kf_second->id_);
                c.Tij = kf_first->optimized_pose_stage_1_.inverse() * kf_second->optimized_pose_stage_1_;
                c.use_pose_stage = 1;

                impl_->loop_candidates_.emplace_back(c);
                check_first = kf_first;
                check_second = kf_second;
            }
        }
    }
}

void LoopClosing::DetectByContinuous() {
    /// GPS良好时不进行搜索
    if (impl_->gps_status_ == 3) {
        return;
    }

    /// 各轨迹的起始点和终点
    std::vector<IdType> endpoints_mapping;
    for (auto &tp : impl_->keyframes_) {
        if (tp.second[0]->bag_type_ == KeyFrameBagType::MAPPING_BAGS) {
            endpoints_mapping.push_back(tp.second.front()->id_);
            endpoints_mapping.push_back(tp.second.back()->id_);
        }
    }

    /// 在建图轨迹终点和起点处进行广域搜索
    const float grid_search_range_step = 1;
    const float grid_search_range = 3;
    const float grid_search_angle_step = 15;

    /// 禁 忌 的 五 重 循 环
    for (int i = 0; i < endpoints_mapping.size() - 1; ++i) {
        for (int j = i + 1; j < endpoints_mapping.size(); ++j) {
            auto e1 = endpoints_mapping[i];
            auto e2 = endpoints_mapping[j];
            for (float dx = -grid_search_range; dx < grid_search_range; dx += grid_search_range_step) {
                for (float dy = -grid_search_range; dy < grid_search_range; dy += grid_search_range_step) {
                    for (float angle = 0; angle < 360; angle += grid_search_angle_step) {
                        /// 搜索之
                        SE3 delta(AngAxisd(angle * M_PI / 180, V3d(0, 0, 1)).matrix(), V3d(dx, dy, 0));

                        auto kf_ref = impl_->keyframes_by_id_mapping_[e1];
                        auto kf = impl_->keyframes_by_id_mapping_[e2];

                        common::LoopCandidate c(kf->id_, kf_ref->id_);
                        c.Tij = delta;  // 假设位置
                        c.use_init_guess = true;
                        c.use_mm_match = false;
                        c.use_pose_stage = 2;
                        impl_->loop_candidates_.emplace_back(c);
                    }
                }
            }
        }
    }
}

void LoopClosing::ComputeRelativeMotionMT() {
    impl_->mt_search_->ComputeConstraint(impl_->loop_candidates_);
    impl_->loop_candidates_ = impl_->mt_search_->GetResults();
    LogAndReport("最终接受的检查点：" + std::to_string(impl_->loop_candidates_.size()));

    int cnt_cloud_loaded = 0;
    for (auto &kfp : impl_->keyframes_by_id_) {
        if (kfp.second->cloud_ != nullptr) {
            cnt_cloud_loaded++;
        }
    }
}

void LoopClosing::SaveLoopCandidates(bool save_all_in_one) {
    if (save_all_in_one) {
        io::SaveLoopCandidates(local_data_path_ + "loops.txt", impl_->loop_candidates_);
    } else {
        io::SaveLoopCandidates(local_data_path_ + "loops_" + std::to_string(impl_->start_id_) + "_" +
                                   std::to_string(impl_->end_id_) + ".txt",
                               impl_->loop_candidates_);
    }

    LogAndReport("已保存回环结果");

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        SaveGemReport(local_data_path_ + gem_report_name_);
    }
}

}  // namespace mapping::pipeline