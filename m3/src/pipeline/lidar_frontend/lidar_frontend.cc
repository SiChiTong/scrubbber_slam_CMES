//
// Created by wangqi on 19-7-19.
//
#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

#include <boost/format.hpp>

#include "core/lidar_matching/feature_matching/feature_matching.h"
#include "core/lidar_matching/lego_loam/lego_loam_interface.h"
#include "core/lidar_matching/lidar_matching.h"
#include "core/lidar_matching/ndt_matching/ndt_matching.h"
#include "io/db_io.h"
#include "io/file_io.h"
#include "pipeline/lidar_frontend/lidar_frontend.h"

namespace mapping::pipeline {

using namespace mapping::io;
using namespace mapping::common;

LidarFrontend::LidarFrontend(const io::YAML_IO &yaml_file, RunMode run_mode, IdType start_kf, IdType end_kf)
    : PipelineContext(run_mode), yaml_file_(yaml_file), start_kf_(start_kf), end_kf_(end_kf) {
    context_name_ = "LidarFrontend";
};

bool LidarFrontend::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }
    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");
    num_lidar_lines_ = yaml_file_.GetValue<int>("velodyne_calib_param", "type");
    matching_type_ = yaml_file_.GetValue<std::string>("lidar_frontend_params", "matching_type");
    debug_save_pcd_ = yaml_file_.GetValue<bool>("save_debug_pcd");

    if (!ReadDataPreparingData()) {
        return false;
    }

    io::RemoveIfExist(local_data_path_ + "/matching_path.txt");

    if (run_mode_ == PipelineContext::RunMode::GEM_EXECUTABLE) {
        local_db_path_ = yaml_file_.GetValue<std::string>("local_db_path");
    }

    return true;
}

LidarFrontend::~LidarFrontend() { LOG(INFO) << "Lidar frontend deconstructed."; }

bool LidarFrontend::Start() {
    SetStatus(ContextStatus::WORKING);
    for (const auto &kfp : keyframes_) {
        ComputeLidarTrajectory(kfp.first);
    }

    if (run_mode_ == RunMode::PIPELINE) {
        SaveResultsForMapping();
    } else {
        SaveResultsForGEM();
    }

    SetStatus(ContextStatus::SUCCEED);
    return true;
}

bool LidarFrontend::SaveResultsForMapping() {
    /// save lidar odom result files.
    io::RemoveIfExist(local_data_path_ + "keyframes.txt");
    if (!io::SaveKeyframePose(local_data_path_ + "keyframes.txt", keyframes_)) {
        report_ += "lidar odom error: save keyframes poses failed.";
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    /// ??????????????????
    io::RemoveIfExist(local_data_path_ + "bef_opt_keyframes.txt");
    io::SaveKeyframePose(local_data_path_ + "bef_opt_keyframes.txt", keyframes_);
    io::RemoveIfExist(local_data_path_ + "matching_degeneracy.txt");
    io::SaveDegeneracyScore(local_data_path_ + "matching_degeneracy.txt", keyframes_);
    io::SaveKeyframeSinglePath(local_data_path_ + "matching_path.txt", keyframes_,
                               io::SaveKeyframePathType::MATCHING_PATH);

    if (debug_save_pcd_) {
        LOG(INFO) << "saving pcd";
        for (auto &kfp : keyframes_) {
            std::string db_path;
            if (kfp.second[0]->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
                db_path = local_data_path_ + "map.db";
                // only save mapping bags
                io::SavePCDWithPose(local_data_path_ + "matching_" + std::to_string(kfp.first) + ".pcd", db_path,
                                    kfp.second, io::SaveKeyframePathType::MATCHING_PATH, 0.5);
                LOG(INFO) << "save pcd " << local_data_path_ + "matching_" + std::to_string(kfp.first) + ".pcd";
            }
        }
    }
    return true;
}

bool LidarFrontend::SaveResultsForGEM() {
    /// save lidar odom result files.
    boost::format fmt("%s/keyframes_%d_%d.txt");

    std::string save_path_nfs = (fmt % local_data_path_ % start_kf_ % end_kf_).str();

    io::RemoveIfExist(save_path_nfs);
    LOG(INFO) << "save results to " << save_path_nfs;

    if (!io::SaveKeyframePose(save_path_nfs, keyframes_)) {
        LOG(ERROR) << "save failed.";
        return false;
    }

    SaveGemReport(local_data_path_ + gem_report_name_);

    return true;
}

bool LidarFrontend::ComputeLidarTrajectory(size_t traj_id) {
    std::shared_ptr<core::LidarMatching> lidar_matcher = nullptr;
    if (matching_type_ == "feature_matching" && num_lidar_lines_ == 16) {
        LogAndReport("?????????????????????????????????");
        core::FeatureMatchingParams params;
        params.LoadFromYAML(yaml_file_);
        lidar_matcher = std::make_shared<core::FeatureMatching>(params);
    }else if (matching_type_ == "feature_matching" && num_lidar_lines_ == 80) {
        //todo ????????????????????????????????????????????????????????????????????????????????????
        LogAndReport("?????????????????????????????????");
        core::LegoLoamParam params;
        params.LoadFromYAML(yaml_file_);
        lidar_matcher = std::make_shared<core::LegoLoamInterface>();
    } else {
        LogAndReport("??????????????????NDT");
        core::NdtMatchingParams params;
        params.LoadFromYAML(yaml_file_);
        lidar_matcher = std::make_shared<core::NdtMatching>(params);
    }

    // ????????????
    auto &keyframes = keyframes_[traj_id];
    std::shared_ptr<io::DB_IO> db_io;

    if (run_mode_ == RunMode::PIPELINE) {
        if (keyframes[0]->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
            db_io = std::make_shared<io::DB_IO>(local_data_path_ + "/map.db");
        } else {
            db_io = std::make_shared<io::DB_IO>(local_data_path_ + "/val.db");
        }
    } else {
        /// GEM ??? DB?????????????????????
        if (keyframes[0]->bag_type_ == common::KeyFrameBagType::MAPPING_BAGS) {
            db_io = std::make_shared<io::DB_IO>(local_db_path_ + "/map.db");
        } else {
            db_io = std::make_shared<io::DB_IO>(local_db_path_ + "/val.db");
        }
    }

    auto start_id = keyframes[0]->id_;
    for (size_t i = 0; i < keyframes.size(); ++i) {
        auto cur_kf = keyframes[i];
        if (run_mode_ == PipelineContext::RunMode::PIPELINE) {
            LOG(INFO) << "computing lidar odom for trajectory " << traj_id << ", kf " << cur_kf->id_ - start_id << "/"
                      << keyframes.size();
        }

        bool load_succ = db_io->ReadSingleKF(cur_kf->id_, cur_kf);
        if (load_succ == false) {
            LOG(ERROR) << "keyframe load failed.";
            continue;
        }

        if (cur_kf->cloud_->empty()) {
            LOG(WARNING) << "kf " << cur_kf->id_ << " cloud data is empty";
        }

        if (i == 0) {
            lidar_matcher->SetStartID(cur_kf->id_);
            lidar_matcher->SetFirstFixedPose(cur_kf->gps_pose_);
            lidar_matcher->SetInputCloud(cur_kf->cloud_, cur_kf->id_);
            lidar_matcher->Run();
        } else {
            auto pre_kf = keyframes[i - 1];
            SE3 predict_pose = pre_kf->dr_pose_.inverse() * cur_kf->dr_pose_;
            lidar_matcher->SetInputCloud(cur_kf->cloud_, cur_kf->id_);
            lidar_matcher->SetPredictPose(predict_pose);
            lidar_matcher->Run();
        }

        cur_kf->UnloadCloud();
    }

    auto matching_poses = lidar_matcher->GetMatchingPoses();
    auto matching_noises = lidar_matcher->GetMatchingNoise();
    auto degeneracy_eigenvalue = lidar_matcher->GetDegeneracyEigenValue();

    std::stringstream traj_info;
    traj_info << "trajectory " << traj_id;

    if (matching_poses.size() != matching_noises.size()) {
        LogAndReport("\\textbf{??????}: ??????" + std::to_string(traj_id) + "????????????????????????????????????");
        return false;
    }

    //??????keyframe???matching_pose
    LOG(INFO) << "updating poses";
    for (auto &matching_pose : matching_poses) {
        keyframes.at(matching_pose.first - start_id)->matching_pose_ = matching_pose.second;
    }

    LOG(INFO) << "updating noise";
    for (auto &matching_noise : matching_noises) {
        keyframes.at(matching_noise.first - start_id)->matching_noise_ = matching_noise.second;
    }

    LOG(INFO) << "updating degen score";
    for (auto &iter : degeneracy_eigenvalue) {
        if (iter.first == start_id) {
            continue;
        }

        keyframes.at(iter.first - 1)->matching_eigen_value_ = iter.second;
    }

    if (run_mode_ == RunMode::PIPELINE) {
        LogAndReport("??????" + std::to_string(traj_id) + "????????????");
    } else if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        LogAndReport("??????" + std::to_string(traj_id) + " : " + std::to_string(start_kf_) + " - " +
                     std::to_string(end_kf_) + " ????????????");
    }
    return true;
}

bool LidarFrontend::ReadDataPreparingData() {
    std::string kf_poses_file = local_data_path_ + "keyframes.txt";

    std::vector<std::shared_ptr<common::KeyFrame>> keyframes;
    if (!io::LoadKeyframes(kf_poses_file, keyframes)) {
        LogAndReport("\\textbf{??????}???????????????????????????????????????" + kf_poses_file);
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    // ???GEM????????????????????????????????????
    if (run_mode_ == PipelineContext::RunMode::GEM_EXECUTABLE) {
        std::vector<std::shared_ptr<common::KeyFrame>> kfs_sel;
        for (auto &kf : keyframes) {
            if (kf->id_ < end_kf_ && kf->id_ >= start_kf_) {
                kfs_sel.push_back(kf);
            }
        }
        LOG(INFO) << "selected keyframes: " << kfs_sel.size() << "/" << keyframes.size();
        kfs_sel.swap(keyframes);
    }

    for (auto &kp : keyframes) {
        if (keyframes_.find(kp->trajectory_id_) == keyframes_.end()) {
            keyframes_.insert({kp->trajectory_id_, {kp}});
        } else {
            keyframes_[kp->trajectory_id_].emplace_back(kp);
        }
    }

    LOG(INFO) << "kfs size = " << keyframes.size();
    if (keyframes_.empty()) {
        LogAndReport("\\textbf{??????}????????????????????????");
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    return true;
}

}  // namespace mapping::pipeline
