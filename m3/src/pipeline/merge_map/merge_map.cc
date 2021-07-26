//
// Created by pengguoqi on 21-1-10.
//

#include "merge_map.h"
#include "io/file_io.h"

#include <glog/logging.h>

namespace mapping {
namespace pipeline {

using namespace mapping::common;

MergeMap::MergeMap(const io::YAML_IO &yaml_file, RunMode run_mode) : yaml_file_(yaml_file), PipelineContext(run_mode) {
    context_name_ = "Merge_Map";
    target_keyframes_end_id_ = 0;
    target_trajectory_end_id_ = 0;
};

MergeMap::~MergeMap() noexcept { LOG(INFO) << "Merge map deconstructed."; }

bool MergeMap::Init() {
    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");
    target_data_path_ = yaml_file_.GetValue<std::string>("origin_map_db_path");
    local_db_path_ = yaml_file_.GetValue<std::string>("local_db_path");
    CopyAndMoveDBFiles();
    if (!io::LoadKeyframes(local_data_path_ + "target_keyframes.txt", target_keyframes_)) {
        LogAndReport("无法读取目标关键帧文件");
        return false;
    }
    if (!io::LoadKeyframes(local_data_path_ + "keyframes.txt", current_keyframes_)) {
        LogAndReport("无法读取当前关键帧文件");
        return false;
    }
    return true;
}

bool MergeMap::Start() {
    SetStatus(ContextStatus::WORKING);
    LOG(INFO) << "Merge Map is working";

    /// 第一步，合并关键帧数据
    MerageKeyFrames();

    /// 第二步，合并db数据
    MerageDB();

    Clear();
    
    LogAndReport("数据合并结束");
    SetStatus(ContextStatus::SUCCEED);
    return true;
}

void MergeMap::CopyAndMoveDBFiles() {
    std::string cmd;
    // 1. 拷贝目标db文件到当前目录下
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        if (!io::PathExists(local_db_path_ + "/target_map.db")) {
            cmd = "cp  " + target_data_path_ + "/map.db " + local_db_path_ + "/target_map.db";
            system(cmd.c_str());
        }
    } else {
        if (!io::PathExists(local_data_path_ + "/target_map.db")) {
            cmd = "cp  " + target_data_path_ + "/map.db " + local_data_path_ + "/target_map.db";
            system(cmd.c_str());
        }
    }

    // 2. 拷贝目标keyframes.txt文件到当前目录下
    cmd = "cp  " + target_data_path_ + "/keyframes.txt " + local_data_path_ + "/target_keyframes.txt";
    system(cmd.c_str());
}

bool MergeMap::MerageDB() {
    std::map<int, SE3> target_map_id_pose;
    std::map<int, SE3> current_map_id_pose;
    ///  单步运行时，不需要修改db数据，
    /// 此情况更多时发生在第一轮优化需要单步调试，修改关键帧数据，所以只需要合并关键帧数据就行
    if (!io::PathExists(local_data_path_ + "/target_map.db")) return false;
    std::shared_ptr<io::DB_IO> target_db_io = std::make_shared<io::DB_IO>(local_data_path_ + "/target_map.db");
    std::shared_ptr<io::DB_IO> current_db_io = std::make_shared<io::DB_IO>(local_data_path_ + "/map.db");
    if (!target_db_io->ReadAllUniqueIdAndPose(target_map_id_pose)) {
        LOG(ERROR) << "Failed to ReadAllUniqueIdAndPose.";
        std::map<int, SE3>().swap(target_map_id_pose);
        target_db_io = nullptr;
        return false;
    }
    if (!current_db_io->ReadAllUniqueIdAndPose(current_map_id_pose)) {
        LOG(ERROR) << "Failed to ReadAllUniqueIdAndPose.";
        std::map<int, SE3>().swap(current_map_id_pose);
        current_db_io = nullptr;
        return false;
    }
    for (const auto &id : current_map_id_pose) {
        KFPtr kf = std::make_shared<KeyFrame>();
        if (current_db_io->ReadSingleKF(id.first, kf)) {
            kf->id_ += target_keyframes_end_id_;
            std::vector<KFPtr> kfps;
            kfps.push_back(kf);
            if (!target_db_io->WritePoseAndCloudToDB(kfps)) {
                LOG(ERROR) << " write smiple pointscloud error!";
                return false;
            }
        } else {
            LOG(ERROR) << " read smiple pointscloud error!";
            continue;
        }
    }

    std::string cmd;
    if (run_mode_ == RunMode::PIPELINE) {
        cmd = "mv  " + local_data_path_ + "/map.db " + local_data_path_ + "/current_map.db";
        system(cmd.c_str());

        cmd = "mv  " + local_data_path_ + "/target_map.db " + local_data_path_ + "/map.db";
        system(cmd.c_str());

    } else if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        cmd = "mv  " + local_db_path_ + "/map.db " + local_db_path_ + "/current_map.db";
        system(cmd.c_str());

        cmd = "mv  " + local_db_path_ + "/target_map.db " + local_db_path_ + "/map.db";
        system(cmd.c_str());
    }

    return true;
}

bool MergeMap::MerageKeyFrames() {
    std::string merge_info_path = local_data_path_ + "merge_info.txt";
    std::string merge_keyframes_path = local_data_path_ + "merge_keyframes.txt";
    target_trajectory_end_id_ = target_keyframes_.size();
    const int target_end_keyframes_size = target_keyframes_[target_trajectory_end_id_ - 1].size();
    target_keyframes_end_id_ =
        target_keyframes_[target_trajectory_end_id_ - 1].at(target_end_keyframes_size - 1)->id_ + 1;
    merge_info_vec_.clear();
    LogAndReport("目标地图轨迹个数 ： " + std::to_string(target_trajectory_end_id_) + ", 关键帧数据 ： " +
                 std::to_string(target_keyframes_end_id_));
    // 记录目标关键帧顶点在后续优化的过程中所有数据为固定顶点
    MergeInfo mi;
    for (int i = 0; i < target_trajectory_end_id_; i++) {
        mi.trajectory_id_ = i;
        mi.vertex_type_ = VertexOptimizationType::FIXED;
        merge_info_vec_.push_back(mi);
    }
    // 将主地图的第二轮优化轨迹赋值给第一轮优化轨迹上
    for (const auto &kfs : target_keyframes_) {
        int trajectory_id = 0;
        for (auto &kf : kfs.second) {
            kf->optimized_pose_stage_1_ = kf->optimized_pose_stage_2_;
        }
    }
    // 合并新增部分的关键帧数据
    int end_id = 0;
    for (const auto &kfs : current_keyframes_) {
        std::vector<common::KFPtr> kf_vec;
        int trajectory_id = 0;
        for (auto &kf : kfs.second) {
            kf->trajectory_id_ += target_trajectory_end_id_;
            trajectory_id = kf->trajectory_id_;
            kf->id_ += target_keyframes_end_id_;
            end_id = kf->id_;
            kf_vec.push_back(kf);
        }
        target_keyframes_.insert({trajectory_id, {kf_vec}});
        kf_vec.clear();
        mi.trajectory_id_ = trajectory_id;
        mi.vertex_type_ = VertexOptimizationType::UNFIXED;
        merge_info_vec_.push_back(mi);
    }
    LogAndReport("目标地图轨迹个数 ： " + std::to_string(current_keyframes_.size()) + ", 关键帧数据 ： " +
                 std::to_string(end_id - target_keyframes_end_id_ + 1));
    io::RemoveIfExist(merge_info_path);
    if (!io::SaveMergeInfo(merge_info_path, merge_info_vec_)) {
        LogAndReport("无法保存合并关联信息文件");
        return false;
    }
    io::RemoveIfExist(merge_keyframes_path);
    if (!io::SaveKeyframePose(merge_keyframes_path, target_keyframes_)) {
        LogAndReport("无法保存合并关键帧文件");
        return false;
    }

    return true;
}

void MergeMap::Clear() {
    target_keyframes_.clear();
    current_keyframes_.clear();
}

}  // namespace pipeline
}  // namespace mapping
