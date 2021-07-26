//
// Created by gaoxiang on 2020/8/10.
//

#include "io/file_io.h"
#include "common/trajectory.h"
#include "common/vehicle_calib_param.h"
#include "db_io.h"
#include "io/db_io.h"
#include "io/yaml_io.h"

#include <glog/logging.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <boost/filesystem.hpp>
#include <fstream>
#include <regex>

#define PCL_IO
namespace mapping::io {

std::mutex pcd_saving_mutex;

bool PathExists(const std::string &file_path) {
    boost::filesystem::path path(file_path);
    return boost::filesystem::exists(path);
}

bool IsDirectory(const std::string &path) { return boost::filesystem::is_directory(path); }

bool FindFile(const std::string &dir, const std::string &filename, std::string &file_path) {
    namespace fs = boost::filesystem;
    fs::path fullpath(dir);
    std::vector<std::string> ret;

    if (!fs::exists(fullpath)) {
        return false;
    }

    bool found = false;
    fs::recursive_directory_iterator end_iter;
    for (fs::recursive_directory_iterator iter(fullpath); iter != end_iter; iter++) {
        if (iter->path().filename() == filename) {
            found = true;
            file_path = iter->path().parent_path().string();
        }
    }
    return found;
}

bool RemoveIfExist(const std::string &path) {
    if (PathExists(path)) {
        LOG(INFO) << "remove " << path;
        system(("rm -f " + path).c_str());
        return true;
    }
    return false;
}

bool SaveKeyframePose(const std::string &save_path, const std::vector<std::shared_ptr<common::KeyFrame>> &keyframes) {
    std::ofstream file;
    file.open(save_path.c_str(), std::ios::out | std::ios::app);
    if (!file) {
        LOG(ERROR) << "failed to open " << save_path;
        return false;
    }

    file << std::fixed;
    auto save_pose = [&file](const SE3 &pose) {
        file << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << " "
             << pose.unit_quaternion().x() << " " << pose.unit_quaternion().y() << " " << pose.unit_quaternion().z()
             << " " << pose.unit_quaternion().w() << " ";
    };

    auto save_noise = [&file](const V6d &noise) {
        for (int i = 0; i < 6; ++i) {
            file << noise[i] << " ";
        }
    };

    for (auto &kf : keyframes) {
        file << std::setprecision(9) << kf->id_ << " "  // id
             << kf->trajectory_id_ << " "               // trajectory_id
             << kf->timestamp_ << " "                   // timestamp
             << int(kf->gps_status_) << " "             // gps status
             << int(kf->bag_type_) << " "               // bag type
             << kf->heading_valid_ << " "               // heading valid
             << kf->gps_inlier_ << " ";                 // gps valid
        save_pose(kf->matching_pose_);
        save_noise(kf->matching_noise_);
        save_pose(kf->dr_pose_);
        save_noise(kf->dr_noise_);
        save_pose(kf->gps_pose_);
        save_noise(kf->gps_noise_);
        save_pose(kf->optimized_pose_stage_1_);
        save_pose(kf->optimized_pose_stage_2_);

        file << kf->matching_eigen_value_;  // 退化分值
        file << std::endl;
    }

    file.close();
    return true;
}

bool SaveKeyframePose(const std::string &save_path,
                      const std::map<IdType, std::vector<std::shared_ptr<common::KeyFrame>>> &keyframes) {
    std::vector<common::KFPtr> kf_vec;
    for (auto &kfp : keyframes) {
        kf_vec.insert(kf_vec.end(), kfp.second.begin(), kfp.second.end());
    }

    return SaveKeyframePose(save_path, kf_vec);
}

bool SaveSpliteResults(const std::string &save_path, const std::map<IdType, std::map<IdType, SE3>> &splite_results) {
    if (splite_results.size() < 1) {
        return false;
    }
    std::ofstream file;
    file.open(save_path.c_str());
    if (!file) {
        LOG(ERROR) << "failed to open " << save_path;
        return false;
    }
    file << std::fixed;

    auto save_result = [&file](const IdType& number, const IdType& id, const SE3 &pose) {
        file << number << " " << pose.translation()[0] << " " << pose.translation()[1] << " "
             << pose.translation()[2] << " " << id;
    };

    for (auto &sr : splite_results) {
        for (auto &r : sr.second) {
            save_result(sr.first, r.first, r.second);
            file << std::endl;
        }
    }
    file.close();
    return true;
}


bool LoadKeyframes(const std::string &file_name, std::vector<std::shared_ptr<common::KeyFrame>> &keyframes) {
    std::ifstream fin(file_name.c_str());
    if (!fin) {
        LOG(ERROR) << "can't open keyframe data file: " << file_name;
        return false;
    }

    auto load_pose = [](std::stringstream &ss) -> SE3 {
        double x, y, z;
        double qx, qy, qz, qw;
        ss >> x >> y >> z >> qx >> qy >> qz >> qw;
        return SE3(Quat(qw, qx, qy, qz), V3d(x, y, z));
    };

    auto load_noise = [](std::stringstream &ss) -> V6d {
        V6d v;
        for (int i = 0; i < 6; ++i) {
            ss >> v[i];
        }
        return v;
    };

    std::string line_data;
    while (std::getline(fin, line_data)) {
        if (line_data.empty()) {
            break;
        }

        std::stringstream data_in(line_data);

        int id, trajectory_id;
        double timestamp;
        int gps_status;
        int bag_type;
        bool heading_valid, gps_valid;

        data_in >> id >> trajectory_id >> timestamp >> gps_status >> bag_type >> heading_valid >> gps_valid;

        auto kf = std::make_shared<common::KeyFrame>();
        kf->id_ = id;
        kf->trajectory_id_ = trajectory_id;
        kf->timestamp_ = timestamp;
        kf->gps_status_ = common::GpsStatusType(gps_status);
        kf->heading_valid_ = heading_valid;
        kf->gps_inlier_ = gps_valid;
        kf->bag_type_ = common::KeyFrameBagType(bag_type);

        kf->matching_pose_ = load_pose(data_in);
        kf->matching_noise_ = load_noise(data_in);

        kf->dr_pose_ = load_pose(data_in);
        kf->dr_noise_ = load_noise(data_in);

        kf->gps_pose_ = load_pose(data_in);
        kf->gps_noise_ = load_noise(data_in);

        kf->optimized_pose_stage_1_ = load_pose(data_in);
        kf->optimized_pose_stage_2_ = load_pose(data_in);

        data_in >> kf->matching_eigen_value_;  // 退化分值
        keyframes.emplace_back(kf);
    }
    fin.close();

    return true;
}

bool LoadKeyframes(const std::string &load_path,
                   std::map<IdType, std::vector<std::shared_ptr<common::KeyFrame>>> &keyframes) {
    std::vector<std::shared_ptr<common::KeyFrame>> kfs;
    if (!LoadKeyframes(load_path, kfs)) {
        return false;
    }

    for (auto &kf : kfs) {
        if (keyframes.find(kf->trajectory_id_) == keyframes.end()) {
            keyframes.insert({kf->trajectory_id_, {kf}});
        } else {
            keyframes[kf->trajectory_id_].emplace_back(kf);
        }
    }
    return true;
}

bool LoadKeyframes(const std::string &load_path, std::map<IdType, std::shared_ptr<common::KeyFrame>> &keyframes) {
    std::vector<std::shared_ptr<common::KeyFrame>> kfs;
    if (!LoadKeyframes(load_path, kfs)) {
        return false;
    }

    for (auto &kf : kfs) {
        keyframes.insert({kf->id_, kf});
    }

    return true;
}

bool LoadOriginInfoByDB(const std::string &map_db_path, common::OriginPointInformation &origin_info) {
    bool re = PathExists(map_db_path);
    if (!re) return false;
    io::DB_IO db_io(map_db_path);
    if (db_io.ReadOriginPointInformationByDB(origin_info)) return true;
    else
        return false;
}

bool SaveMergeInfo(const std::string &save_path, const common::MergeInfoVec &merge_info_vec) {
    std::ofstream file;
    file.open(save_path.c_str(), std::ios::out | std::ios::app);
    if (!file) {
        LOG(ERROR) << "failed to open " << save_path;
        return false;
    }
    file << std::fixed;
    for (auto &mi : merge_info_vec) {
        file << mi.trajectory_id_ << " ";
        if (mi.vertex_type_ == common::VertexOptimizationType::FIXED) {
            file << 0;
        } else {
            file << 1;
        }
        file << std::endl;
    }
    file.close();
    return true;
}

bool LoadMergeInfo(const std::string &file_name, common::MergeInfoVec &merge_info_vec) {
    std::ifstream fin(file_name.c_str());
    if (!fin) {
        LOG(ERROR) << "can't open merge data file: " << file_name;
        return false;
    }

    std::string line_data;
    while (std::getline(fin, line_data)) {
        if (line_data.empty()) {
            break;
        }

        std::stringstream data_in(line_data);

        int trajectory_id;
        int vertex_type;

        common::MergeInfo mi;

        data_in >> trajectory_id >> vertex_type;
        mi.trajectory_id_ = trajectory_id;
        if (0 == vertex_type) {
            mi.vertex_type_ = common::VertexOptimizationType::FIXED;
        } else {
            mi.vertex_type_ = common::VertexOptimizationType::UNFIXED;
        }
        merge_info_vec.push_back(mi);
    }
    fin.close();
    return true;
}

bool SaveDegeneracyScore(const std::string &save_path,
                         const std::map<IdType, std::vector<std::shared_ptr<common::KeyFrame>>> &keyframes) {
    std::ofstream file;
    file.open(save_path.c_str());
    if (!file) {
        return false;
    }

    file << std::fixed;
    for (auto &kfp : keyframes) {
        for (const auto &kf : kfp.second) {
            float eigen_value = kf->matching_eigen_value_;
            bool is_degeneracy = false;

            if (eigen_value < 100) {
                is_degeneracy = true;
            }
            file << kf->id_ << " " << eigen_value << " " << is_degeneracy << std::endl;
        }
    }

    file.close();
    return true;
}

bool SaveKeyframeSinglePath(const std::string &path,
                            const std::map<IdType, std::vector<std::shared_ptr<common::KeyFrame>>> &keyframes,
                            SaveKeyframePathType save_type) {
    std::ofstream file(path);
    if (!file) {
        LOG(ERROR) << "cannot save to " << path;
        return false;
    }

    for (auto &kfp : keyframes) {
        for (auto &kf : kfp.second) {
            SE3 pose;
            switch (save_type) {
                case SaveKeyframePathType::GPS_PATH:
                    pose = kf->gps_pose_;
                    break;
                case SaveKeyframePathType::DR_PATH:
                    pose = kf->dr_pose_;
                    break;
                case SaveKeyframePathType::MATCHING_PATH:
                    pose = kf->matching_pose_;
                    break;
                case SaveKeyframePathType::OPTI_PATH_STAGE_1:
                    pose = kf->optimized_pose_stage_1_;
                    break;
                case SaveKeyframePathType::OPTI_PATH_STAGE_2:
                    pose = kf->optimized_pose_stage_2_;
                    break;
                default:
                    break;
            }

            V3d t = pose.translation();
            Quat q = pose.unit_quaternion();
            file << std::setprecision(18) << kf->id_ << " " << kf->timestamp_ << " " << std::setprecision(9) << t.x()
                 << " " << t.y() << " " << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
                 << kf->trajectory_id_;

            if (save_type == SaveKeyframePathType::GPS_PATH) {
                file << " " << kf->gps_inlier_ && kf->gps_status_ == common::GpsStatusType::GNSS_FIXED_SOLUTION;
            }

            file << std::endl;
        }
    }
    file.close();
    return true;
}

bool SavePCDWithPoseTest(const std::string &path, const std::string &db_path,
                         const std::vector<std::shared_ptr<common::KeyFrame>> &keyframes,
                         SaveKeyframePathType pose_type, double delta_x, double delta_y, double resolution) {
    RemoveIfExist(path);
    io::DB_IO db_io(db_path);
    common::PointCloudType::Ptr opt_map_points_filtered(new common::PointCloudType());
    pcl::VoxelGrid<common::PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    common::PointCloudPtr global_cloud(new common::PointCloudType);

    for (auto &kf : keyframes) {
        SE3 pose;
        switch (pose_type) {
            case SaveKeyframePathType::GPS_PATH:
                pose = kf->gps_pose_;
                break;
            case SaveKeyframePathType::DR_PATH:
                pose = kf->dr_pose_;
                break;
            case SaveKeyframePathType::MATCHING_PATH:
                pose = kf->matching_pose_;
                break;
            case SaveKeyframePathType::OPTI_PATH_STAGE_1:
                pose = kf->optimized_pose_stage_1_;
                break;
            case SaveKeyframePathType::OPTI_PATH_STAGE_2:
                pose = kf->optimized_pose_stage_2_;
                pose.translation()[0] += delta_x;
                pose.translation()[1] += delta_y;
                break;
            default:
                break;
        }

        if (!db_io.ReadSingleKF(kf->id_, kf, false) || kf->cloud_->empty()) {
            continue;
        }

        common::PointCloudPtr cloud_trans(new common::PointCloudType);
        common::PointCloudPtr cloud_trans_filtered(new common::PointCloudType);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix().cast<float>());

        voxel_grid_filter.setInputCloud(cloud_trans);
        voxel_grid_filter.filter(*cloud_trans_filtered);

        *global_cloud += *cloud_trans_filtered;
        kf->UnloadCloud();
    }

    if (global_cloud->empty()) {
        return false;
    }

    common::PointCloudPtr global_cloud_filtered(new common::PointCloudType);
    voxel_grid_filter.setInputCloud(global_cloud);
    voxel_grid_filter.filter(*global_cloud_filtered);

    if (!global_cloud_filtered->empty()) {
        LOG(INFO) << "saving pcd";
        std::unique_lock<std::mutex> pcd_saving_lock(pcd_saving_mutex);
        pcl::io::savePCDFileBinaryCompressed(path, *global_cloud_filtered);
        LOG(INFO) << "point cloud saved at " << path << ", pts: " << global_cloud_filtered->points.size();
    }

    return true;
}

bool SavePCDWithPose(const std::string &path, const std::string &db_path,
                     const std::vector<std::shared_ptr<common::KeyFrame>> &keyframes, SaveKeyframePathType pose_type,
                     double resolution) {
    RemoveIfExist(path);
    io::DB_IO db_io(db_path);
    common::PointCloudType::Ptr opt_map_points_filtered(new common::PointCloudType());
    pcl::VoxelGrid<common::PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    common::PointCloudPtr global_cloud(new common::PointCloudType);

    for (auto &kf : keyframes) {
        SE3 pose;
        switch (pose_type) {
            case SaveKeyframePathType::GPS_PATH:
                pose = kf->gps_pose_;
                // pose = SE3(Quat(kf->optimized_pose_stage_2_.unit_quaternion().w(), kf->optimized_pose_stage_2_.unit_quaternion().x(), kf->optimized_pose_stage_2_.unit_quaternion().y(), kf->optimized_pose_stage_2_.unit_quaternion().z()), kf->gps_pose_.translation());
                break;
            case SaveKeyframePathType::DR_PATH:
                pose = kf->dr_pose_;
                break;
            case SaveKeyframePathType::MATCHING_PATH:
                pose = kf->matching_pose_;
                break;
            case SaveKeyframePathType::OPTI_PATH_STAGE_1:
                pose = kf->optimized_pose_stage_1_;
                break;
            case SaveKeyframePathType::OPTI_PATH_STAGE_2:
                pose = kf->optimized_pose_stage_2_;
                break;
            default:
                break;
        }

        if (!db_io.ReadSingleKF(kf->id_, kf, false) || kf->cloud_->empty()) {
            continue;
        }
        
        //test point save
        // std::string pcd_path = "/home/idriver/data/huilongguan_0309/pcd2/" + std::to_string(kf->id_)+".pcd";
        // pcl::PointCloud<pcl::PointXYZI>::Ptr test_pcd(new pcl::PointCloud<pcl::PointXYZI>());        
        // test_pcd->header = kf->cloud_->header;
        // test_pcd->points.resize(kf->cloud_->points.size());
        // for (int i = 0; i < kf->cloud_->points.size(); i ++) {
        //     auto point = kf->cloud_->points[i];
        //     test_pcd->points[i].x = point.x;
        //     test_pcd->points[i].y = point.y;
        //     test_pcd->points[i].z = point.z;
        //     test_pcd->points[i].intensity = point.intensity;
        // }

        // pcl::io::savePCDFileBinaryCompressed(pcd_path, *test_pcd);
        // pcl::PointCloud<pcl::PointXYZI>::Ptr test_pcd_out(new pcl::PointCloud<pcl::PointXYZI>());
        // pcl::transformPointCloud(*test_pcd, *test_pcd_out, pose.matrix().cast<float>());
        // pcl::io::savePCDFileBinaryCompressed(pcd_path, *test_pcd_out);

        common::PointCloudPtr cloud_trans(new common::PointCloudType);
        common::PointCloudPtr cloud_trans_filtered(new common::PointCloudType);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix().cast<float>());

        voxel_grid_filter.setInputCloud(cloud_trans);
        voxel_grid_filter.filter(*cloud_trans_filtered);

        *global_cloud += *cloud_trans_filtered;
        kf->UnloadCloud();
    }

    if (global_cloud->empty()) {
        return false;
    }

    common::PointCloudPtr global_cloud_filtered(new common::PointCloudType);
    voxel_grid_filter.setInputCloud(global_cloud);
    voxel_grid_filter.filter(*global_cloud_filtered);

    if (!global_cloud_filtered->empty()) {
        LOG(INFO) << "saving pcd";
        std::unique_lock<std::mutex> pcd_saving_lock(pcd_saving_mutex);
        pcl::io::savePCDFileBinaryCompressed(path, *global_cloud_filtered);
        LOG(INFO) << "point cloud saved at " << path << ", pts: " << global_cloud_filtered->points.size();
    }

    return true;
}

bool SavePCDWithMapDB(const std::string &path, const std::string &db_path,
                      double resolution) {
    RemoveIfExist(path);
    io::DB_IO db_io(db_path);
    common::PointCloudType::Ptr opt_map_points_filtered(new common::PointCloudType());
    pcl::VoxelGrid<common::PointType> voxel_grid_filter;
    voxel_grid_filter.setLeafSize(resolution, resolution, resolution);

    common::PointCloudPtr global_cloud(new common::PointCloudType);
    std::map<int, SE3> map_id_pose;
    db_io.ReadAllUniqueIdAndPose(map_id_pose);
    for (auto &pose_id : map_id_pose) {
        std::shared_ptr<common::KeyFrame> kf = std::make_shared<common::KeyFrame>();
        SE3 pose = pose_id.second;
        if (!db_io.ReadSingleKF(pose_id.first, kf) || kf->cloud_->empty()) {
            continue;
        }

        common::PointCloudPtr cloud_trans(new common::PointCloudType);
        common::PointCloudPtr cloud_trans_filtered(new common::PointCloudType);
        pcl::transformPointCloud(*kf->cloud_, *cloud_trans, pose.matrix().cast<float>());

        voxel_grid_filter.setInputCloud(cloud_trans);
        voxel_grid_filter.filter(*cloud_trans_filtered);

        *global_cloud += *cloud_trans_filtered;
        kf->UnloadCloud();
    }

    if (global_cloud->empty()) {
        return false;
    }

    common::PointCloudPtr global_cloud_filtered(new common::PointCloudType);
    voxel_grid_filter.setInputCloud(global_cloud);
    voxel_grid_filter.filter(*global_cloud_filtered);

    if (!global_cloud_filtered->empty()) {
        LOG(INFO) << "saving pcd";
        std::unique_lock<std::mutex> pcd_saving_lock(pcd_saving_mutex);
        pcl::io::savePCDFileBinaryCompressed(path, *global_cloud_filtered);
        LOG(INFO) << "point cloud saved at " << path << ", pts: " << global_cloud_filtered->points.size();
    }

    return true;
}

bool SaveSinglePCDWithPose(const std::string &path, const std::string &db_path,
                           const std::shared_ptr<common::KeyFrame> &keyframe, SaveKeyframePathType pose_type) {
    SE3 pose;
    switch (pose_type) {
        case SaveKeyframePathType::GPS_PATH:
            pose = keyframe->gps_pose_;
            break;
        case SaveKeyframePathType::DR_PATH:
            pose = keyframe->dr_pose_;
            break;
        case SaveKeyframePathType::MATCHING_PATH:
            pose = keyframe->matching_pose_;
            break;
        case SaveKeyframePathType::OPTI_PATH_STAGE_1:
            pose = keyframe->optimized_pose_stage_1_;
            break;
        case SaveKeyframePathType::OPTI_PATH_STAGE_2:
            pose = keyframe->optimized_pose_stage_2_;
            break;
        default:
            pose = keyframe->optimized_pose_stage_2_;
            break;
    }
    RemoveIfExist(path);
    io::DB_IO db_io(db_path);
    common::PointCloudPtr cloud_trans(new common::PointCloudType);
    if (!db_io.ReadSingleKF(keyframe->id_, keyframe, false) || keyframe->cloud_->empty()) {
        return false;
    }
    pcl::transformPointCloud(*keyframe->cloud_, *cloud_trans, pose.matrix().cast<float>());
    keyframe->UnloadCloud();
    if (!cloud_trans->empty()) {
        LOG(INFO) << "saving pcd";
        std::unique_lock<std::mutex> pcd_saving_lock(pcd_saving_mutex);
        pcl::io::savePCDFileBinaryCompressed(path, *cloud_trans);
        LOG(INFO) << "point cloud saved at " << path << ", pts: " << cloud_trans->points.size();
    }
    return true;
}

bool SaveLoopCandidates(const std::string &path, const std::vector<common::LoopCandidate> &loop_candidates,
                        bool append) {
    if (!append) {
        RemoveIfExist(path);
    }

    std::ofstream file;
    if (append) {
        file.open(path, std::ios::app);
    } else {
        file.open(path);
    }

    if (!file) {
        LOG(ERROR) << "cannot save to " << path;
        return false;
    }

    for (auto &lc : loop_candidates) {
        V3d t = lc.Tij.translation();
        Quat q = lc.Tij.unit_quaternion();
        file << std::setprecision(9) << lc.kfid_first << " " << lc.kfid_second << " " << t.x() << " " << t.y() << " "
             << t.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
    }

    file.close();
    return true;
}

bool SaveLoopCandidatesScore(const std::string &path, const std::vector<common::LoopCandidate> &loop_candidates,
                             std::map<IdType, common::KFPtr> keyframes, bool remove) {
    if (remove) {
        RemoveIfExist(path);
    }

    std::ofstream file(path, std::ios::app);
    if (!file) {
        LOG(ERROR) << "cannot save to " << path;
        return false;
    }

    for (auto &lc : loop_candidates) {
        file << std::setprecision(9) << keyframes[lc.kfid_first]->optimized_pose_stage_2_.translation()[0] << " "
             << keyframes[lc.kfid_first]->optimized_pose_stage_2_.translation()[1] << " " << lc.score << std::endl;
    }

    file.close();
    return true;
}

bool LoadLoopCandidates(const std::string &path, std::vector<common::LoopCandidate> &loop_candidates) {
    std::ifstream fin(path);
    if (!fin) {
        LOG(ERROR) << "cannot load file " << path;
        return false;
    }

    std::string line_data;
    while (std::getline(fin, line_data)) {
        if (line_data.empty()) {
            break;
        }
        std::stringstream ss(line_data);
        IdType first_id, second_id;
        double x, y, z;
        double qx, qy, qz, qw;
        ss >> first_id >> second_id >> x >> y >> z >> qx >> qy >> qz >> qw;

        common::LoopCandidate lc(first_id, second_id);
        lc.Tij = SE3(Quat(qw, qx, qy, qz), V3d(x, y, z));
        loop_candidates.emplace_back(lc);
    }

    fin.close();

    return true;
}

bool LoadLoopCandidatesScore(const std::string &path, std::vector<common::NdtOriginData> &score_ndt) {
    std::ifstream fin(path);
    if (!fin) {
        LOG(ERROR) << "cannot load file " << path;
        return false;
    }

    std::string line_data;
    while (std::getline(fin, line_data)) {
        if (line_data.empty()) {
            break;
        }
        std::stringstream ss(line_data);
        double score = 0, x = 0, y = 0;
        ss >> x >> y >> score;

        score_ndt.emplace_back(x, y, score);
    }

    fin.close();

    return true;
}

bool SaveTrajectoryBagName(const std::string &path,
                           const std::map<IdType, std::shared_ptr<common::Trajectory>> &trajectory_map) {
    RemoveIfExist(path);
    std::ofstream fout(path);
    if (!fout) {
        LOG(ERROR) << "cannot save at " << path;
        return false;
    }

    for (auto &tp : trajectory_map) {
        for (auto &bp : tp.second->bag_files) {
            fout << tp.first << " " << bp.first << " " << bp.second << " " << tp.second->is_mapping_bag_ << std::endl;
        }
    }
    fout.close();
    return true;
}

bool LoadTrajectoryBagName(const std::string &path,
                           std::map<IdType, std::shared_ptr<common::Trajectory>> &trajectory_map,
                           bool only_validation_bags) {
    std::ifstream fin(path);
    if (!fin) {
        LOG(ERROR) << "cannot load file " << path;
        return false;
    }

    std::string line_data;
    while (std::getline(fin, line_data)) {
        if (line_data.empty()) {
            break;
        }
        std::stringstream ss(line_data);

        int traj_id, bag_id;
        std::string bag_name;
        bool is_mapping_bag;

        ss >> traj_id >> bag_id >> bag_name >> is_mapping_bag;

        if (only_validation_bags && is_mapping_bag) {
            continue;
        }

        auto iter = trajectory_map.find(traj_id);
        bool has_traj = iter != trajectory_map.end();

        if (has_traj) {
            // add bag name
            iter->second->bag_files.insert({bag_id, bag_name});
        } else {
            // add new trajectory
            auto traj = std::make_shared<common::Trajectory>();
            traj->trajectory_id = traj_id;
            traj->bag_files.insert({bag_id, bag_name});
            traj->is_mapping_bag_ = is_mapping_bag;

            trajectory_map.insert({traj_id, traj});

            static std::string pre_bag_name_map, pre_bag_name_val;
            if (bag_name.at(0) == 'd') {
                pre_bag_name_map = bag_name;
            } else if (bag_name.at(0) == 'z') {
                pre_bag_name_val = bag_name;
            }
        }
    }

    fin.close();

    return true;
}

bool SaveGpsError(const std::string &path, const std::vector<double> &gps_chi2) {
    RemoveIfExist(path);
    std::ofstream fout(path);
    if (!fout) {
        LOG(ERROR) << "cannot save at " << path;
        return false;
    }

    for (auto &chi2 : gps_chi2) {
        fout << chi2 << std::endl;
    }
    fout.close();
    return true;
}

bool SaveGpsError(const std::string &path, const std::vector<V6d> &gps_res) {
    RemoveIfExist(path);
    std::ofstream fout(path);
    if (!fout) {
        LOG(ERROR) << "cannot save at " << path;
        return false;
    }

    for (auto &res : gps_res) {
        fout << res[0] << " " << res[1] << " " << res[2] << " " << res[3] << " " << res[4] << " " << res[5]
             << std::endl;
    }

    fout.close();
    return true;
}

FileNameInfo ParseBagName(const std::string &name) {
    const std::regex rex(R"(.*/ConvertData/+([d,z].*?)_(ENFORCE_)?(\d*_)*(part(\d+)*_)*V.bag)");
    std::smatch sm;
    FileNameInfo info;
    if (!std::regex_match(name, sm, rex)) {
        LOG(ERROR) << "cannot match bag name: " << name;
        return info;
    }

    info.parse_success = true;
    info.bag_name = sm[1];
    info.has_part = !std::string(sm[4]).empty();
    if (!sm[5].str().empty()) {
        info.part_num = std::atoi(sm[5].str().c_str());
    }
    info.bag_file_path = name;

    return info;
}

double GetBagFileTime(const std::string &name) {
    const std::regex rex(R"(.*/ConvertData/+([d,z].*?)_(ENFORCE_)?((\d*)_)*(part(\d+)*_)*V.bag)");
    std::smatch sm;

    if (!std::regex_match(name, sm, rex)) {
        LOG(ERROR) << "cannot match bag name: " << name;
        return -1.0;
    }

    std::string time_string = sm[4];
    //判断sm[4]是否为纯数字
    const std::regex rex_dig(R"(\d+)");
    if (!std::regex_match(time_string, rex_dig)) {
        LOG(ERROR) << "cannot match bag name: " << time_string;
        return -1.0;
    }

    double time = std::stod(time_string);
    LOG(INFO) << std::setprecision(16) << "bag time is: " << time << " " << name;
    return time;
}

bool KeyFramesPoints(std::string db_path, std::vector<std::shared_ptr<common::KeyFrame>> &pointcloud_vec) {
    std::vector<int> frames_read_fail_id;
    io::DB_IO db_io(db_path);

    if (!db_io.ReadAllKF(pointcloud_vec, frames_read_fail_id)) {
        LOG(ERROR) << " read pose and cloud failed! (map loader builder) ";
        return false;
    }

    return true;
}

bool LoadVehicleParamsFromYAML(const io::YAML_IO &yaml, common::VehicleCalibrationParam &vehicle_params) {
    vehicle_params.antenna_x = yaml.GetValue<double>("gps_params", "ant_x");
    vehicle_params.antenna_y = yaml.GetValue<double>("gps_params", "ant_y");
    vehicle_params.antenna_angle = yaml.GetValue<double>("gps_params", "ant_angle");
    vehicle_params.perception_lidar_x_offset_top_center = yaml.GetValue<double>("velodyne_calib_param", "xoffset");
    vehicle_params.perception_lidar_y_offset_top_center = yaml.GetValue<double>("velodyne_calib_param", "yoffset");
    vehicle_params.perception_lidar_z_offset_top_center = yaml.GetValue<double>("velodyne_calib_param", "zoffset");
    vehicle_params.perception_lidar_roll_top_center = yaml.GetValue<double>("velodyne_calib_param", "roll");
    vehicle_params.perception_lidar_pitch_top_center = yaml.GetValue<double>("velodyne_calib_param", "pitch");
    vehicle_params.perception_lidar_yaw_top_center = yaml.GetValue<double>("velodyne_calib_param", "yaw");
    vehicle_params.static_gyro_var = yaml.GetValue<double>("dr_params", "static_gyro_var");
    vehicle_params.static_acc_var = yaml.GetValue<double>("dr_params", "static_acc_var");
    vehicle_params.odom_ratio_left = yaml.GetValue<double>("dr_params", "ratio_left");
    vehicle_params.odom_ratio_right = yaml.GetValue<double>("dr_params", "ratio_right");
    vehicle_params.lidar_type = yaml.GetValue<int>("velodyne_calib_param", "type");
    vehicle_params.lidar_factory = yaml.GetValue<std::string>("velodyne_calib_param", "factory");
    return true;
}

bool LoadGpsInfo(const std::string &path, std::vector<GpsInfo> &gps_info) {
    std::ifstream fin(path);
    if (!fin) {
        LOG(ERROR) << "cannot load file " << path;
        return false;
    }

    std::string line_data;
    while (std::getline(fin, line_data)) {
        if (line_data.empty()) {
            break;
        }
        std::stringstream ss(line_data);
        IdType id = 0;
        double timestamp = 0;
        double x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 0;
        IdType trajectory_id = 0;
        bool fixed_inlier = false;
        ss >> id >> timestamp >> x >> y >> z >> qx >> qy >> qz >> qw >> trajectory_id >> fixed_inlier;
        gps_info.emplace_back(id, fixed_inlier);
    }

    fin.close();

    return true;
}

}  // namespace mapping::io
