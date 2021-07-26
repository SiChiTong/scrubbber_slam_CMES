//
// Created by gaoxiang on 2020/8/14.
//

#include <pcl/io/pcd_io.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <Eigen/Geometry>
#include <regex>

#include <glog/logging.h>
#include <src/common/car_type.h>

#include "common/function_point.h"
#include "common/mapping_point_types.h"
#include "core/coordinate_transform/gps_trans.h"
#include "core/dead_reckoning/dr.h"
#include "core/dead_reckoning/dr_params.h"
#include "io/file_io.h"
#include "io/xml_io.h"
#include "pipeline/check_in/file_filter.h"
#include "pipeline/dr_frontend/dr_frontend.h"
#include "src/common/trajectory.h"
#include "tools/perception_interface/interface.h"
#include "tools/pointcloud_convert/hesai_pc_convertor.h"
#include "tools/pointcloud_convert/suteng_pc_convertor.h"
#include "tools/pointcloud_convert/suteng_irad_pc_convertor.h"

namespace mapping::pipeline {

using common::KeyFrame;
using common::Trajectory;
constexpr double GPS_MAX = 100000;

common::GpsStatusType GpsMsgToGpsStatusType(const GpsMsgPtr &gps_msg) { return common::GpsStatusType(gps_msg->status); }

// tools
// DR elements 转 SE3
inline SE3 DRElementsToSE3(const core::DrElements &dr_pose) {
    using AA = Eigen::AngleAxisd;
    double real_yaw = M_PI * 0.5 - dr_pose.attitude(2);
    if (real_yaw < -M_PI) {
        real_yaw += 2.0 * M_PI;
    } else {
        real_yaw -= 2.0 * M_PI;
    }
    V3d att = dr_pose.attitude;

    return SE3(AA(att[0], V3d::UnitX()) * AA(att[1], V3d::UnitY()) * AA(real_yaw, V3d::UnitZ()), dr_pose.position);
}

inline tools::HesaiConf SetHeSaiLidarConfig(const tools::VelodyneConfig &params) {
    tools::HesaiConf config;
    config.xoffset = params.xoffset;
    config.yoffset = params.yoffset;
    config.zoffset = params.zoffset;
    config.roll = params.roll;
    config.pitch = params.pitch;
    config.yaw = params.yaw;
    std::unordered_map<int, std::string> LIDAR_TYPE = {{16, "PandarXT-16"}, {32, "PandarXT-32"}, {64, "Pandar64"}};
    config.lidar_type = LIDAR_TYPE[params.type];
    return config;
}

DRFrontend::DRFrontend(const io::YAML_IO &yaml_file, RunMode run_mode)
    : yaml_file_(yaml_file), PipelineContext(run_mode) {
    context_name_ = "DRFrontend";
    ResetDR();
}

DRFrontend::~DRFrontend() { LOG(INFO) << "DR frontend deconstructed"; }

bool DRFrontend::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }
    debug_save_pcd_ = yaml_file_.GetValue<bool>("save_debug_pcd");
    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");

    if (!io::PathExists(local_data_path_)) {
        system(("mkdir -p " + local_data_path_).c_str());
    }

    perception_interface_ = std::make_shared<tools::PerceptionInterface>(yaml_file_);

    FilesFilter files_filter(local_data_path_);
    files_filter.FilterAllFolderUnderParent();
    origin_files_ = files_filter.GetFilteredFiles();

    if (origin_files_.converted_bag_files.empty()) {
        LogAndReport("DR未找到有效的数据包");
        return false;
    }

    LogAndReport("有效数据包：" + std::to_string(origin_files_.converted_bag_files.size()));

    common::OriginPointInformation origin_info;

    origin_info.map_origin_x = yaml_file_.GetValue<double>("map_origin_param", "map_origin_x");
    origin_info.map_origin_y = yaml_file_.GetValue<double>("map_origin_param", "map_origin_y");
    origin_info.map_origin_z = yaml_file_.GetValue<double>("map_origin_param", "map_origin_z");
    origin_info.map_origin_zone = yaml_file_.GetValue<int>("map_origin_param", "map_origin_zone");
    origin_info.is_southern = yaml_file_.GetValue<bool>("map_origin_param", "is_southern");
    double ant_x = yaml_file_.GetValue<double>("gps_params", "ant_x");
    double ant_y = yaml_file_.GetValue<double>("gps_params", "ant_y");
    double ant_angle = yaml_file_.GetValue<double>("gps_params", "ant_angle");
    ant_angle_ = ant_angle;
    std::string car_type = yaml_file_.GetValue<std::string>("car_type");
    if (car_type == "lads") {
        car_type_ = common::CarType::LADS;
    }
    lidar_manufacturer_ = yaml_file_.GetValue<std::string>("velodyne_calib_param", "factory");
    local_db_path_ = yaml_file_.GetValue<std::string>("local_db_path");

    if (run_mode_ == PipelineContext::RunMode::GEM_EXECUTABLE && !io::PathExists(local_db_path_)) {
        system(("mkdir -p " + local_db_path_).c_str());
    }

    auto const &velodyne_config = perception_interface_->GetVelodyneConfig();
    is_irad_ = velodyne_config.is_irad;

    if (lidar_manufacturer_ == "velodyne") {
    } else if (lidar_manufacturer_ == "suteng") {
        // 创建速腾的格式转换器
        assert(io::PathExists(local_data_path_ + "localization_map_collection.yaml"));
        suteng_lidar_convertor_ = std::make_shared<tools::SuTengPcConvertor>(local_data_path_ + "suteng.yaml");
        suteng_lidar_irad_convertor_ = std::make_shared<tools::SuTengIRADPcConvertor>();
        if (is_irad_) {
            common::VehicleCalibrationParam config;
            config.perception_lidar_x_offset_top_center = velodyne_config.xoffset;
            config.perception_lidar_y_offset_top_center = velodyne_config.yoffset;
            config.perception_lidar_z_offset_top_center = velodyne_config.zoffset;
            config.perception_lidar_roll_top_center = velodyne_config.roll;
            config.perception_lidar_pitch_top_center = velodyne_config.pitch;
            config.perception_lidar_yaw_top_center = velodyne_config.yaw;
            config.pre_rot_axis_0 = velodyne_config.pre_rot_axis_0;
            config.pre_rot_axis_1 = velodyne_config.pre_rot_axis_1;
            config.pre_rot_axis_2 = velodyne_config.pre_rot_axis_2;
            config.pre_rot_degree_0 = velodyne_config.pre_rot_degree_0;
            config.pre_rot_degree_1 = velodyne_config.pre_rot_degree_1;
            config.pre_rot_degree_2 = velodyne_config.pre_rot_degree_2;
            if (suteng_lidar_irad_convertor_->Init(config) != 0) {
                LogAndReport("速腾驱动初始化失败");
                return false;
            }
            std::string suteng_type = suteng_lidar_irad_convertor_->GetSutengLidarType();
            int suteng_lidar_type = CheckSutengLidarType(suteng_type);
            if (velodyne_config.type != suteng_lidar_type) {
                LOG(WARNING) << "Suteng lidar type is " << suteng_lidar_type << " , not " << velodyne_config.type;
            } else {
                LOG(INFO) << "Real suteng type is: RS" << std::to_string(suteng_lidar_type);
            }
        } else {
            if (suteng_lidar_convertor_->Init() != 0) {
                LogAndReport("速腾驱动初始化失败");
                return false;
            }
            std::string suteng_type = suteng_lidar_convertor_->GetSutengLidarType();
            int suteng_lidar_type = CheckSutengLidarType(suteng_type);
            if (velodyne_config.type != suteng_lidar_type) {
                LOG(WARNING) << "Suteng lidar type is " << suteng_lidar_type << " , not " << velodyne_config.type;
            } else {
                LOG(INFO) << "Real suteng type is: RS" << std::to_string(suteng_lidar_type);
            }
        }
    } else if (lidar_manufacturer_ == "hesai") {
        // 创建禾赛的格式转换器
        tools::VelodyneConfig params;
        params.LoadFromYAML(yaml_file_);
        tools::HesaiConf config = SetHeSaiLidarConfig(params);
        hesai_lidar_convertor_ = std::make_shared<tools::HeSaiPcConvertor>(config);
    }

    gps_trans_ = std::make_shared<core::GpsTransform>(
        V3d(origin_info.map_origin_x, origin_info.map_origin_y, origin_info.map_origin_z), ant_x, ant_y, ant_angle);

    if (velodyne_config.type == 16) {
        dist_th_ = 0.5;
        LOG(INFO) << "running 16 lines lidar";
        LogAndReport("使用16线雷达");
    } else if (velodyne_config.type == 32) {
        dist_th_ = 1.0;
        LOG(INFO) << "running 32 lines lidar";
        LogAndReport("使用32线雷达");
    } else if (velodyne_config.type == 64) {
        dist_th_ = 1.0;
        LOG(INFO) << "running 64 lines lidar";
        LogAndReport("使用64线雷达");
    }else if (velodyne_config.type == 80) {
        LOG(INFO) << "running 80 lines lidar";
        dist_th_ = 1.2;
        LogAndReport("使用80线雷达");
    } else {
        LogAndReport("未知的雷达类型：" + std::to_string(velodyne_config.type));
        return false;
    }

    if (run_mode_ == RunMode::PIPELINE) {
        io::RemoveIfExist(local_data_path_ + "/map*.db");
        io::RemoveIfExist(local_data_path_ + "/val*.db");
    }

    io::RemoveIfExist(local_data_path_ + "/keyframes*.txt");
    io::RemoveIfExist(local_data_path_ + "/dr_path*.txt");
    io::RemoveIfExist(local_data_path_ + "/gps_path*.txt");

    return true;
}

void DRFrontend::ResetDR() {
    core::DrParams dr_params;
    dr_params.LoadFromYAML(yaml_file_);
    dr_ = std::make_shared<core::DeadReckoning>(dr_params);
    dr_->SetAzimuth(dr_angle_ini_set_);
    dr_->SetPosition(V3d::Zero());
}

bool DRFrontend::Start() {
    SetStatus(ContextStatus::WORKING);

    // 检查自动分包，创建轨迹
    LogAndReport("解析数据包名称中");
    if (!io::LoadTrajectoryBagName(local_data_path_ + "trajectory_info.txt", trajectory_, false)) {
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    // 计算每条轨迹上的DR，分割关键帧
    LogAndReport("计算DR轨迹");
    if (!ComputeDRTrajectory()) {
        LogAndReport("计算DR轨迹失败");
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    // 从BAG中读取点云，分配给关键帧并写入DB
    LogAndReport("创建数据库并写入点云");
    std::string db_path_mapping;
    std::string db_path_validation;

    if (run_mode_ == PipelineContext::RunMode::PIPELINE) {
        db_path_validation = local_data_path_ + "val.db";  // 验证包的DB
        db_path_mapping = local_data_path_ + "map.db";     // 建图包的DB
    } else {
        db_path_validation = local_db_path_ + "val.db";  // 验证包的DB
        db_path_mapping = local_db_path_ + "map.db";     // 建图包的DB
    }

    io::DB_IO db_io_mapping(db_path_mapping);
    io::DB_IO db_io_validation(db_path_validation);

    if (lidar_manufacturer_ == "velodyne") {
        AssignPointCloudDataForVelodyne(db_io_mapping, db_io_validation);
    } else if (lidar_manufacturer_ == "hesai") {
        AssignPointCloudDataForHesai(db_io_mapping, db_io_validation);
    } else if (lidar_manufacturer_ == "suteng") {
        if (is_irad_) {
            AssignPointCloudDataForSutengIRAD(db_io_mapping, db_io_validation);
        } else {
            AssignPointCloudDataForSuteng(db_io_mapping, db_io_validation);
        }
    }

    // Save Keyframe poses
    SaveResults();
    LogAndReport("DR前端完成");

    SetStatus(ContextStatus::SUCCEED);
    return true;
}

void DRFrontend::SaveResults() {
    // collect keyframes
    LOG(INFO) << "saving results";
    std::map<IdType, std::vector<common::KFPtr>> kfs_map;
    int cnt_keyframes = 0;
    for (auto &t : trajectory_) {
        kfs_map.insert({t.first, {}});
        for (auto kf : t.second->keyframes_) {
            kfs_map[t.first].emplace_back(kf);
            cnt_keyframes++;
        }
    }

    /// 存储到keyframes.txt中
    LOG(INFO) << "keyframes: " << cnt_keyframes;
    io::RemoveIfExist(local_data_path_ + "keyframes.txt");
    io::SaveKeyframePose(local_data_path_ + "keyframes.txt", kfs_map);

    /// 存储到dr_path中
    io::SaveKeyframeSinglePath(local_data_path_ + "dr_path.txt", kfs_map, io::SaveKeyframePathType::DR_PATH);
    /// GPS path
    io::SaveKeyframeSinglePath(local_data_path_ + "gps_path.txt", kfs_map, io::SaveKeyframePathType::GPS_PATH);

    LOG(INFO) << "saving dr pcd";
    if (debug_save_pcd_) {
        for (auto &t : trajectory_) {
            std::string db_path;
            if (t.second->is_mapping_bag_) {
                db_path = local_data_path_ + "map.db";
            } else {
                db_path = local_data_path_ + "val.db";
            }

            io::SavePCDWithPose(local_data_path_ + "dr_" + std::to_string(t.first) + ".pcd", db_path,
                                t.second->keyframes_, io::SaveKeyframePathType::DR_PATH);
        }
    }

    /// set task info
    task_info_.valid_trajs = trajectory_.size();
    task_info_.total_keyframes = cnt_keyframes;
    task_info_.area = task_info_.length * 8.5;

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        yaml_file_.SetValue("task_info", "length", task_info_.length);
        yaml_file_.SetValue("task_info", "area", task_info_.area);
        yaml_file_.SetValue("task_info", "total_keyframes", task_info_.total_keyframes);
        yaml_file_.Save();

        SaveGemReport(local_data_path_ + gem_report_name_);
    }
}

void DRFrontend::AssignPointCloudDataForVelodyne(io::DB_IO &db_io_mapping, io::DB_IO &db_io_validation) {
    const double time_th = 20;
    int cnt_loaded_kfs = 0;
    int cnt_all_kfs = 0;

    auto save_and_unload_pcd = [this](std::vector<common::KFPtr> &kfs, io::DB_IO &db_io) {
        for (auto &kf : kfs) {
            assert(kf->cloud_->points.empty() == false);
        }

        db_io.WritePoseAndCloudToDB(kfs);
        for (auto &kf : kfs) {
            kf->UnloadCloud();
        }
        kfs.clear();
    };

    for (auto &tp : trajectory_) {
        auto kf_iter = tp.second->keyframes_.begin();
        cnt_all_kfs += tp.second->keyframes_.size();

        /// packets 会多读一些，便于做缓存
        std::deque<PacketsMsgPtr> packets_msg_queue;
        for (auto bag_files : tp.second->bag_files) {
            if (kf_iter == tp.second->keyframes_.end()) {
                break;
            }

            // 寻找为kf_iter关联的激光数据
            rosbag::Bag bag;
            bag.open(bag_files.second, rosbag::bagmode::Read);

            for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
                if (kf_iter == tp.second->keyframes_.end()) {
                    break;
                }

                auto pm = m.instantiate<PacketsMsg>();
                if (pm == nullptr) continue;

                packets_msg_queue.push_back(pm);

                double t = (*kf_iter)->timestamp_;

                if (pm->header.stamp.toSec() < t + time_th) {
                    // 继续读
                    continue;
                }

                if (FindSynLidarDataForVelodyne(*kf_iter, packets_msg_queue)) {
                    if (tp.second->is_mapping_bag_) {
                        keyframes_cache_mapping_.push_back(*kf_iter);
                    } else {
                        keyframes_cache_validation_.push_back(*kf_iter);
                    }
                    kf_iter++;
                    cnt_loaded_kfs++;
                }

                while (packets_msg_queue.front()->header.stamp.toSec() < t - time_th) {
                    packets_msg_queue.pop_front();
                }

                if (keyframes_cache_mapping_.size() > keyframes_cache_max_size_) {
                    // clean cache
                    save_and_unload_pcd(keyframes_cache_mapping_, db_io_mapping);
                }

                if (keyframes_cache_validation_.size() > keyframes_cache_max_size_) {
                    // clean cache
                    save_and_unload_pcd(keyframes_cache_validation_, db_io_validation);
                }
            }
        }

        /// 为剩下的keyframes找激光数据
        while (kf_iter != tp.second->keyframes_.end()) {
            if (FindSynLidarDataForVelodyne(*kf_iter, packets_msg_queue)) {
                if (tp.second->is_mapping_bag_) {
                    keyframes_cache_mapping_.push_back(*kf_iter);
                } else {
                    keyframes_cache_validation_.push_back(*kf_iter);
                }

                kf_iter++;
                cnt_loaded_kfs++;
            } else {
                break;
            }
        }

        // clean cache
        save_and_unload_pcd(keyframes_cache_mapping_, db_io_mapping);
        save_and_unload_pcd(keyframes_cache_validation_, db_io_validation);
    }

    LOG(INFO) << "extracted keyframes/total: " << cnt_loaded_kfs << "/" << cnt_all_kfs;
}

void DRFrontend::AssignPointCloudDataForSuteng(io::DB_IO &db_io_mapping, io::DB_IO &db_io_validation) {
    const double time_th = 20;
    int cnt_loaded_kfs = 0;
    int cnt_all_kfs = 0;

    auto save_and_unload_pcd = [this](std::vector<common::KFPtr> &kfs, io::DB_IO &db_io) {
        db_io.WritePoseAndCloudToDB(kfs);
        for (auto &kf : kfs) {
            kf->UnloadCloud();
        }
        kfs.clear();
    };

    for (auto &tp : trajectory_) {
        auto kf_iter = tp.second->keyframes_.begin();
        cnt_all_kfs += tp.second->keyframes_.size();

        /// packets 会多读一些，便于做缓存
        std::deque<SuTengPacketsMsgPtr> packets_msg_queue;
        std::deque<SuTengScanMsgPtr> scan_msg_queue;

        for (auto bag_files : tp.second->bag_files) {
            if (kf_iter == tp.second->keyframes_.end()) {
                break;
            }

            // 寻找为kf_iter关联的激光数据
            rosbag::Bag bag;
            bag.open(bag_files.second, rosbag::bagmode::Read);

            for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
                if (kf_iter == tp.second->keyframes_.end()) {
                    break;
                }

                double t = (*kf_iter)->timestamp_;

                auto pm = m.instantiate<SuTengScanMsg>();
                if (pm) {
                    scan_msg_queue.push_back(pm);
                    if (pm->header.stamp.toSec() < t + time_th) {
                        // 继续读
                        continue;
                    }
                }

                auto pp = m.instantiate<SuTengPacketsMsg>();
                if (pp) {
                    packets_msg_queue.push_back(pp);
                    if (pp->stamp.toSec() < t + time_th) {
                        // 继续读
                        continue;
                    }
                }

                if (pm == nullptr && pp == nullptr) {
                    continue;
                }

                if (FindSynLidarDataForSuteng(*kf_iter, scan_msg_queue, packets_msg_queue)) {
                    if ((*kf_iter)->cloud_->empty()) {
                        // 驱动失败了
                        kf_iter = tp.second->keyframes_.erase(kf_iter);
                    } else {
                        if (tp.second->is_mapping_bag_) {
                            keyframes_cache_mapping_.push_back(*kf_iter);
                        } else {
                            keyframes_cache_validation_.push_back(*kf_iter);
                        }
                        kf_iter++;
                        cnt_loaded_kfs++;
                    }
                }

                while (packets_msg_queue.empty() == false && packets_msg_queue.front()->stamp.toSec() < t - time_th) {
                    packets_msg_queue.pop_front();
                }

                while (scan_msg_queue.empty() == false && scan_msg_queue.front()->header.stamp.toSec() < t - time_th) {
                    scan_msg_queue.pop_front();
                }

                if (keyframes_cache_mapping_.size() > keyframes_cache_max_size_) {
                    // clean cache
                    save_and_unload_pcd(keyframes_cache_mapping_, db_io_mapping);
                }

                if (keyframes_cache_validation_.size() > keyframes_cache_max_size_) {
                    // clean cache
                    save_and_unload_pcd(keyframes_cache_validation_, db_io_validation);
                }
            }
        }

        /// 为剩下的keyframes找激光数据
        while (kf_iter != tp.second->keyframes_.end()) {
            if (FindSynLidarDataForSuteng(*kf_iter, scan_msg_queue, packets_msg_queue)) {
                if ((*kf_iter)->cloud_->empty()) {
                    // 驱动失败了
                    kf_iter = tp.second->keyframes_.erase(kf_iter);
                } else {
                    if (tp.second->is_mapping_bag_) {
                        keyframes_cache_mapping_.push_back(*kf_iter);
                    } else {
                        keyframes_cache_validation_.push_back(*kf_iter);
                    }

                    kf_iter++;
                    cnt_loaded_kfs++;
                }
            } else {
                kf_iter = tp.second->keyframes_.erase(kf_iter);
            }
        }

        // clean cache
        save_and_unload_pcd(keyframes_cache_mapping_, db_io_mapping);
        save_and_unload_pcd(keyframes_cache_validation_, db_io_validation);
    }

    LOG(INFO) << "extracted keyframes/total: " << cnt_loaded_kfs << "/" << cnt_all_kfs;
}

void DRFrontend::AssignPointCloudDataForSutengIRAD(io::DB_IO &db_io_mapping, io::DB_IO &db_io_validation) {
    const double time_th = 20;
    int cnt_loaded_kfs = 0;
    int cnt_all_kfs = 0;

    auto save_and_unload_pcd = [this](std::vector<common::KFPtr> &kfs, io::DB_IO &db_io) {
        db_io.WritePoseAndCloudToDB(kfs);
        for (auto &kf : kfs) {
            if (debug_save_pcd_ && kf->cloud_->points.empty() == false) {
                pcl::io::savePCDFileBinary(local_data_path_ + "keyframe_" + std::to_string(kf->id_) + ".pcd",
                                           *kf->cloud_);
            }
            kf->UnloadCloud();
        }
        kfs.clear();
    };

    for (auto &tp : trajectory_) {
        auto kf_iter = tp.second->keyframes_.begin();
        cnt_all_kfs += tp.second->keyframes_.size();

        std::deque<SuTengIRADPacketsMsgPtr> scan_msg_queue;

        for (auto bag_files : tp.second->bag_files) {
            if (kf_iter == tp.second->keyframes_.end()) {
                break;
            }

            // 寻找为kf_iter关联的激光数据
            rosbag::Bag bag;
            bag.open(bag_files.second, rosbag::bagmode::Read);

            for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
                if (kf_iter == tp.second->keyframes_.end()) {
                    break;
                }

                double t = (*kf_iter)->timestamp_;

                auto pm = m.instantiate<SuTengIRADPacketsMsg>();
                if (pm) {
                    scan_msg_queue.push_back(pm);
                    if (pm->header.stamp.toSec() < t + time_th) {
                        // 继续读
                        continue;
                    }
                }

                if (pm == nullptr) {
                    continue;
                }

                if (FindSynLidarDataForSuteng(*kf_iter, scan_msg_queue)) {
                    if ((*kf_iter)->cloud_->empty()) {
                        // 驱动失败了
                        kf_iter = tp.second->keyframes_.erase(kf_iter);
                    } else {
                        if (tp.second->is_mapping_bag_) {
                            keyframes_cache_mapping_.push_back(*kf_iter);
                        } else {
                            keyframes_cache_validation_.push_back(*kf_iter);
                        }
                        kf_iter++;
                        cnt_loaded_kfs++;
                    }
                }

                while (scan_msg_queue.empty() == false && scan_msg_queue.front()->header.stamp.toSec() < t - time_th) {
                    scan_msg_queue.pop_front();
                }

                if (keyframes_cache_mapping_.size() > keyframes_cache_max_size_) {
                    // clean cache
                    save_and_unload_pcd(keyframes_cache_mapping_, db_io_mapping);
                }

                if (keyframes_cache_validation_.size() > keyframes_cache_max_size_) {
                    // clean cache
                    save_and_unload_pcd(keyframes_cache_validation_, db_io_validation);
                }
            }
        }

        /// 为剩下的keyframes找激光数据
        while (kf_iter != tp.second->keyframes_.end()) {
            if (FindSynLidarDataForSuteng(*kf_iter, scan_msg_queue)) {
                if ((*kf_iter)->cloud_->empty()) {
                    // 驱动失败了
                    kf_iter = tp.second->keyframes_.erase(kf_iter);
                } else {
                    if (tp.second->is_mapping_bag_) {
                        keyframes_cache_mapping_.push_back(*kf_iter);
                    } else {
                        keyframes_cache_validation_.push_back(*kf_iter);
                    }

                    kf_iter++;
                    cnt_loaded_kfs++;
                }
            } else {
                kf_iter = tp.second->keyframes_.erase(kf_iter);
            }
        }

        // clean cache
        save_and_unload_pcd(keyframes_cache_mapping_, db_io_mapping);
        save_and_unload_pcd(keyframes_cache_validation_, db_io_validation);
    }

    LOG(INFO) << "extracted keyframes/total: " << cnt_loaded_kfs << "/" << cnt_all_kfs;
} 


void DRFrontend::AssignPointCloudDataForHesai(io::DB_IO &db_io_mapping, io::DB_IO &db_io_validation) {
    const double time_th = 20;
    int cnt_loaded_kfs = 0;
    int cnt_all_kfs = 0;

    auto save_and_unload_pcd = [this](std::vector<common::KFPtr> &kfs, io::DB_IO &db_io) {
        db_io.WritePoseAndCloudToDB(kfs);
        for (auto &kf : kfs) {
            if (debug_save_pcd_ && kf->cloud_->points.empty() == false) {
                pcl::io::savePCDFileBinary(local_data_path_ + "keyframe_" + std::to_string(kf->id_) + ".pcd",
                                           *kf->cloud_);
            }
            kf->UnloadCloud();
        }
        kfs.clear();
    };

    for (auto &tp : trajectory_) {
        auto kf_iter = tp.second->keyframes_.begin();
        cnt_all_kfs += tp.second->keyframes_.size();

        std::deque<HeSaiScanMsgPtr> scan_msg_queue;

        for (auto bag_files : tp.second->bag_files) {
            if (kf_iter == tp.second->keyframes_.end()) {
                break;
            }

            // 寻找为kf_iter关联的激光数据
            rosbag::Bag bag;
            bag.open(bag_files.second, rosbag::bagmode::Read);

            for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
                if (kf_iter == tp.second->keyframes_.end()) {
                    break;
                }

                double t = (*kf_iter)->timestamp_;

                auto pm = m.instantiate<HeSaiScanMsg>();
                if (pm) {
                    scan_msg_queue.push_back(pm);
                    if (pm->header.stamp.toSec() < t + time_th) {
                        // 继续读
                        continue;
                    }
                }

                if (pm == nullptr) {
                    continue;
                }

                if (FindSynLidarDataForHesai(*kf_iter, scan_msg_queue)) {
                    if ((*kf_iter)->cloud_->empty()) {
                        // 驱动失败了
                        kf_iter = tp.second->keyframes_.erase(kf_iter);
                    } else {
                        if (tp.second->is_mapping_bag_) {
                            keyframes_cache_mapping_.push_back(*kf_iter);
                        } else {
                            keyframes_cache_validation_.push_back(*kf_iter);
                        }
                        kf_iter++;
                        cnt_loaded_kfs++;
                    }
                }

                while (scan_msg_queue.empty() == false && scan_msg_queue.front()->header.stamp.toSec() < t - time_th) {
                    scan_msg_queue.pop_front();
                }

                if (keyframes_cache_mapping_.size() > keyframes_cache_max_size_) {
                    // clean cache
                    save_and_unload_pcd(keyframes_cache_mapping_, db_io_mapping);
                }

                if (keyframes_cache_validation_.size() > keyframes_cache_max_size_) {
                    // clean cache
                    save_and_unload_pcd(keyframes_cache_validation_, db_io_validation);
                }
            }
        }

        /// 为剩下的keyframes找激光数据
        while (kf_iter != tp.second->keyframes_.end()) {
            if (FindSynLidarDataForHesai(*kf_iter, scan_msg_queue)) {
                if ((*kf_iter)->cloud_->empty()) {
                    // 驱动失败了
                    kf_iter = tp.second->keyframes_.erase(kf_iter);
                } else {
                    if (tp.second->is_mapping_bag_) {
                        keyframes_cache_mapping_.push_back(*kf_iter);
                    } else {
                        keyframes_cache_validation_.push_back(*kf_iter);
                    }

                    kf_iter++;
                    cnt_loaded_kfs++;
                }
            } else {
                kf_iter = tp.second->keyframes_.erase(kf_iter);
            }
        }

        // clean cache
        save_and_unload_pcd(keyframes_cache_mapping_, db_io_mapping);
        save_and_unload_pcd(keyframes_cache_validation_, db_io_validation);
    }

    LOG(INFO) << "extracted keyframes/total: " << cnt_loaded_kfs << "/" << cnt_all_kfs;
} 

bool DRFrontend::FindSynLidarDataForVelodyne(std::shared_ptr<common::KeyFrame> kf,
                                             const std::deque<PacketsMsgPtr> &packets_buffer) {
    if (packets_buffer.empty()) {
        return false;
    }

    const double delta_time_th = 20;
    auto best_iter = FindClosest<PacketsMsgPtr>(kf->timestamp_, packets_buffer,
                                                [](PacketsMsgPtr p) { return p->header.stamp.toSec(); });
    if (best_iter == packets_buffer.end()) {
        return false;
    }

    /// 建立三个packets给感知接口以获取点云
    auto first_iter = best_iter;
    auto second_iter = best_iter;
    auto third_iter = best_iter;
    if (best_iter != packets_buffer.begin()) {
        first_iter--;
    }
    if (third_iter != packets_buffer.end() - 1) {
        third_iter++;
    }

    std::vector<PacketsMsgPtr> three_packets{*first_iter, *second_iter, *third_iter};

    perception_interface_->SetDrPose(kf_to_dr_poses_[kf]);
    common::PointCloudPtr cloud(new common::PointCloudType);
    if (perception_interface_->GetVelodyneConfig().type == 16) {
        perception_interface_->PointConvertAndHeightProcess(three_packets, cloud);
    } else if (perception_interface_->GetVelodyneConfig().type == 32) {
        perception_interface_->ProcessScan32(three_packets[1], cloud);
    } else if (perception_interface_->GetVelodyneConfig().type == 64) {
        perception_interface_->ProcessScan64(three_packets[1], cloud);
    }

    ResetKeyFrameTimeAndPose(three_packets[1]->header.stamp.toSec(), kf);

    if (cloud->points.empty()) {
        LOG(INFO) << "cloud from perception interface is empty: " << kf->id_;
        return false;
    }

    kf->cloud_ = cloud;
    return true;
}

bool DRFrontend::FindSynLidarDataForSuteng(std::shared_ptr<common::KeyFrame> kf,
                                           const std::deque<SuTengScanMsgPtr> &scan_buffer,
                                           const std::deque<SuTengPacketsMsgPtr> &packets_buffer) {
    if (packets_buffer.empty() || scan_buffer.empty()) {
        LOG(INFO) << "packets_buffer: " << packets_buffer.size() << " " << scan_buffer.size();
        return false;
    }

    const double delta_time_th = 20;

    if (packets_buffer.front()->stamp.toSec() > kf->timestamp_ + delta_time_th) {
        /// 窗口比kf晚
        return false;
    }
    if (packets_buffer.back()->stamp.toSec() < kf->timestamp_ - delta_time_th) {
        // kf比窗口晚
        return false;
    }
    if (scan_buffer.front()->header.stamp.toSec() > kf->timestamp_ + delta_time_th) {
        /// 窗口比kf晚
        return false;
    }
    if (scan_buffer.back()->header.stamp.toSec() < kf->timestamp_ - delta_time_th) {
        // kf比窗口晚
        return false;
    }

    LOG(INFO) << "FindSynLidarDataForSuteng: " << packets_buffer.size() << " " << scan_buffer.size();

    /// find the closest one
    auto iter_packets = FindClosest<SuTengPacketsMsgPtr>(kf->timestamp_, packets_buffer,
                                                         [](SuTengPacketsMsgPtr msg) { return msg->stamp.toSec(); });
    auto iter_scan = FindClosest<SuTengScanMsgPtr>(kf->timestamp_, scan_buffer,
                                                   [](SuTengScanMsgPtr msg) { return msg->header.stamp.toSec(); });
    LOG(INFO) << "Find Closest Ptr!";
    if (iter_packets != packets_buffer.end() && iter_scan != scan_buffer.end()) {
        if (kf_to_dr_poses_.find(kf) != kf_to_dr_poses_.end()) {
            suteng_lidar_convertor_->SetDrPose(kf_to_dr_poses_[kf]);
        }

        common::PointCloudPtr cloud_out(new common::PointCloudType);
        suteng_lidar_convertor_->Convert(**iter_scan, **iter_packets, cloud_out);

        if (cloud_out->points.size() < 1) {
            LOG(INFO) << "Second Converter!";
            suteng_lidar_convertor_->Convert(**iter_scan, **iter_packets, cloud_out);
        }
        // LOG(INFO) << "Points size: " << cloud_out->points.size();

        ResetKeyFrameTimeAndPose((**iter_scan).header.stamp.toSec(), kf);

        kf->cloud_ = cloud_out;
        return true;
    } else {
        LOG(INFO) << "Can not find closest packet or scan!";
    }

    return false;
}

bool DRFrontend::FindSynLidarDataForSuteng(std::shared_ptr<common::KeyFrame> kf,
                                           const std::deque<SuTengIRADPacketsMsgPtr> &scan_buffer) {
    const double delta_time_th = 20;

    if (scan_buffer.front()->header.stamp.toSec() > kf->timestamp_ + delta_time_th) {
        /// 窗口比kf晚
        LOG(INFO) << "scan_buffer.front()->header.stamp.toSec() > kf->timestamp_ + delta_time_th";
        return false;
    }
    if (scan_buffer.back()->header.stamp.toSec() < kf->timestamp_ - delta_time_th) {
        // kf比窗口晚
        LOG(INFO) << "scan_buffer.back()->header.stamp.toSec() < kf->timestamp_ - delta_time_th";
        return false;
    }

    /// find the closest one
    auto iter_scan = FindClosest<SuTengIRADPacketsMsgPtr>(kf->timestamp_, scan_buffer,
                                                  [](SuTengIRADPacketsMsgPtr msg) { return msg->header.stamp.toSec(); });

    if (iter_scan != scan_buffer.end()) {
        if (kf_to_dr_poses_.find(kf) != kf_to_dr_poses_.end()) {
            suteng_lidar_irad_convertor_->SetDrPose(kf_to_dr_poses_[kf]);
        }

        common::PointCloudPtr cloud_out(new common::PointCloudType);
        suteng_lidar_irad_convertor_->Convert(**iter_scan, cloud_out);

        if (cloud_out->points.size() < 1) {
            LOG(INFO) << "Second Converter!";
            suteng_lidar_irad_convertor_->Convert(**iter_scan, cloud_out);
        }

        ResetKeyFrameTimeAndPose((**iter_scan).header.stamp.toSec(), kf);

        kf->cloud_ = cloud_out;
        return true;
    }

    return false;
}

bool DRFrontend::FindSynLidarDataForHesai(std::shared_ptr<common::KeyFrame> kf,
                                          const std::deque<HeSaiScanMsgPtr> &scan_buffer) {
    const double delta_time_th = 20;

    if (scan_buffer.front()->header.stamp.toSec() > kf->timestamp_ + delta_time_th) {
        /// 窗口比kf晚
        LOG(INFO) << "scan_buffer.front()->header.stamp.toSec() > kf->timestamp_ + delta_time_th";
        return false;
    }
    if (scan_buffer.back()->header.stamp.toSec() < kf->timestamp_ - delta_time_th) {
        // kf比窗口晚
        LOG(INFO) << "scan_buffer.back()->header.stamp.toSec() < kf->timestamp_ - delta_time_th";
        return false;
    }

    /// find the closest one
    auto iter_scan = FindClosest<HeSaiScanMsgPtr>(kf->timestamp_, scan_buffer,
                                                  [](HeSaiScanMsgPtr msg) { return msg->header.stamp.toSec(); });

    if (iter_scan != scan_buffer.end()) {
        if (kf_to_dr_poses_.find(kf) != kf_to_dr_poses_.end()) {
            hesai_lidar_convertor_->SetDrPose(kf_to_dr_poses_[kf]);
        }

        common::PointCloudPtr cloud_out(new common::PointCloudType);
        hesai_lidar_convertor_->Convert(**iter_scan, cloud_out);
        kf->cloud_ = cloud_out;
        return true;
    }

    return false;
}

bool DRFrontend::ParseBagsByName() {
    /// 匹配包名
    bool has_success = false;
    for (auto &bag_file : origin_files_.converted_bag_files) {
        auto info = io::ParseBagName(bag_file);
        if (!info.parse_success) {
            LogAndReport("无法匹配包名：" + bag_file);
            continue;
        }

        has_success = true;
        bool is_z_bag = info.bag_name[0] == 'z';

        auto traj = FindStartTrajectory(info.bag_name);
        int part_n = info.part_num;  // part_num 为空时，默认为0

        if (traj == nullptr) {
            /// 不是分包，或者是分包但总包还没被解析出来（因为文件名顺序不一定是按时间先后排的）
            traj = std::make_shared<Trajectory>(traj_id_, info.bag_name);
            traj->bag_files.insert({part_n, bag_file});
            traj->is_mapping_bag_ = !is_z_bag;
            trajectory_.insert({traj_id_, traj});
            traj_id_++;
        } else {
            traj->bag_files.insert({part_n, bag_file});
        }
    }

    for (auto &tp : trajectory_) {
        std::stringstream ss;
        ss << "轨迹" << tp.first << "数据包：\n";
        for (auto &bag_name_pair : tp.second->bag_files) {
            std::string bag_name = bag_name_pair.second;
            bag_name.find_last_of('/');
            std::string file_name = bag_name.substr(bag_name.find_last_of('/') + 1);
            ss << "第" << bag_name_pair.first << "段：" << file_name << std::endl;
        }

        LogAndReport(ss.str());
    }

    LogAndReport("数据包数量：" + std::to_string(origin_files_.converted_bag_files.size()) + "，轨迹数量：" +
                 std::to_string(trajectory_.size()));
    return has_success;
}

std::shared_ptr<Trajectory> DRFrontend::FindStartTrajectory(const std::string &bag_name) {
    for (auto tp : trajectory_) {
        if (tp.second->bag_name == bag_name) {
            return tp.second;
        }
    }
    return nullptr;
}

bool DRFrontend::ComputeDRTrajectory() {
    for (auto &tp : trajectory_) {
        /// Collect IMU and Odom messages
        dr_path_.clear();
        gps_path_.clear();

        auto t = tp.second;  // 轨迹
        CollectMessage(t);

        if(t->gps_msgs_.size() > 0){
            if(GpsMsgToGpsStatusType(t->gps_msgs_.begin()->second) == common::GpsStatusType::GNSS_FIXED_SOLUTION){
                dr_angle_ini_set_ = t->gps_msgs_.begin()->second->heading + ant_angle_;
            }
        }

        ResetDR();

        auto imu_iter = t->imu_msgs_.begin();
        auto odom_iter = t->odom_msgs_.begin();

        while (!(imu_iter == t->imu_msgs_.end() && odom_iter == t->odom_msgs_.end())) {
            if (imu_iter == t->imu_msgs_.end()) {
                ProcessOdom(odom_iter->second);
                odom_iter++;
            } else if (odom_iter == t->odom_msgs_.end()) {
                ProcessIMU(imu_iter->second);
                imu_iter++;
            } else {
                if (imu_iter->first < odom_iter->first) {
                    ProcessIMU(imu_iter->second);
                    imu_iter++;
                } else {
                    ProcessOdom(odom_iter->second);
                    odom_iter++;
                }
            }
        }

        if (dr_path_.empty()) {
            LogAndReport("\\color{red}{错误}无法计算轨迹" + std::to_string(tp.first) + "的DR路径");
            return false;
        }

        LogAndReport("轨迹" + std::to_string(tp.first) + " DR 位姿：" + std::to_string(dr_path_.size()));

        LOG(INFO) << "computing gps path for traj " << tp.first;
        for (auto &gps_msg : t->gps_msgs_) {
            M4d gps_eigen = M4d::Identity();
            gps_trans_->GpsToEigen(*gps_msg.second, gps_eigen);
            if (fabs(gps_eigen(0, 3)) > GPS_MAX || fabs(gps_eigen(1, 3)) > GPS_MAX || fabs(gps_eigen(2, 3)) > GPS_MAX) {
                continue;
            }

            gps_path_.insert(
                {gps_msg.first,
                 {GpsMsgToGpsStatusType(gps_msg.second), SE3(gps_eigen), bool(gps_msg.second->is_heading_valid)}});
        }
        LogAndReport("轨迹" + std::to_string(tp.first) + " GPS 位姿：" + std::to_string(gps_path_.size()));

        // 抽取关键帧
        ExtractKeyFrames(tp.second);
        LogAndReport("轨迹" + std::to_string(tp.first) + " 关键帧数：" + std::to_string(tp.second->keyframes_.size()));

        all_gps_path_.insert(std::make_pair(tp.first, gps_path_));
        // 计算长度
        if (tp.second->is_mapping_bag_) {
            double length = 0;
            V3d last_trans = tp.second->keyframes_.front()->dr_pose_.translation();
            for (auto &kf : tp.second->keyframes_) {
                length += (last_trans - kf->dr_pose_.translation()).norm();
                last_trans = kf->dr_pose_.translation();
            }

            task_info_.length += length;
            LOG(INFO) << "trajectory " << tp.first << " length: " << length;
        }
    }
    return true;
}

void DRFrontend::ExtractKeyFrames(std::shared_ptr<Trajectory> trajectory) {
    /// 将基于dr_path来构建trajectory
    if (dr_path_.empty()) {
        LOG(WARNING) << "dr path is empty.";
        return;
    }

    LOG(INFO) << "keyframe theshold: " << dist_th_ << " " << angle_dist_th_ << " " << time_diff_th_;

    auto last_dr_iter = dr_path_.begin();
    for (auto iter = dr_path_.begin(); iter != dr_path_.end(); ++iter) {
        double dis = (last_dr_iter->second.translation().head<2>() - iter->second.translation().head<2>()).norm();
        double angle = (last_dr_iter->second.so3().inverse() * iter->second.so3()).log().norm() * 180 / M_PI;
        double dt = fabs(last_dr_iter->first - iter->first);

        if (iter == dr_path_.begin() || dis > dist_th_ || angle > angle_dist_th_ || dt > time_diff_th_) {
            // 增加一个keyframe
            auto new_kf = std::make_shared<KeyFrame>();
            new_kf->id_ = keyframe_id_++;
            new_kf->trajectory_id_ = trajectory->trajectory_id;
            new_kf->dr_pose_ = iter->second;
            new_kf->timestamp_ = iter->first;

            // find gps pose
            common::GpsPoseStatus gps_pose_status;
            bool has_gps = FindSynGpsPose(iter->first, gps_path_, gps_pose_status);
            if (has_gps) {
                new_kf->gps_status_ = gps_pose_status.status_;
                new_kf->gps_pose_ = gps_pose_status.pose_;
                new_kf->heading_valid_ = gps_pose_status.heading_valid_;
            }

            // find adjacent dr poses
            auto adj_dr_iter_min = iter;
            auto adj_dr_iter_max = iter;
            for (int i = 0; i < 10; ++i) {
                if (adj_dr_iter_min != dr_path_.begin()) {
                    adj_dr_iter_min--;
                }
                if (adj_dr_iter_max != dr_path_.end()) {
                    adj_dr_iter_max++;
                }
            }

            std::vector<common::TimedPose> pose_buffer;
            for (auto adj_iter = adj_dr_iter_min; adj_iter != adj_dr_iter_max; ++adj_iter) {
                pose_buffer.push_back({adj_iter->first, adj_iter->second});
            }

            kf_to_dr_poses_.insert({new_kf, pose_buffer});
            if (trajectory->is_mapping_bag_) {
                new_kf->bag_type_ = common::KeyFrameBagType::MAPPING_BAGS;
            } else {
                new_kf->bag_type_ = common::KeyFrameBagType::VALIDATION_BAGS;
            }

            trajectory->keyframes_.push_back(new_kf);
            last_dr_iter = iter;
        }
    }
}

bool DRFrontend::FindSynGpsPose(double pose_time, const std::map<double, common::GpsPoseStatus> &gps_path,
                                common::GpsPoseStatus &best_match) {
    if (gps_path.empty()) {
        return false;
    }

    if (pose_time > gps_path.rbegin()->first) {
        return false;
    }

    auto match_iter = gps_path.begin();
    for (auto iter = gps_path.begin(); iter != gps_path.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;

        if (iter->first < pose_time && next_iter->first >= pose_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    match_iter_n++;
    // assert(match_iter_n != gps_path.end());
    if (match_iter_n == gps_path.end()) {
        return false;
    }

    double dt = match_iter_n->first - match_iter->first;
    double s = (pose_time - match_iter->first) / dt;  // s=0 时为第一帧，s=1时为next
    auto best_iter = s < 0.5 ? match_iter : match_iter_n;

    if (std::fabs(match_iter_n->first - match_iter->first) > 0.5) {
        LOG(INFO) << "gps time is jumping: " << match_iter_n->first - match_iter->first;
        return false;
    }
    if (std::fabs(best_iter->first - pose_time) > 0.2) {
        LOG(INFO) << "gps time and scan time is jumping: " << best_iter->first - pose_time;
        return false;
    }

    best_match.pose_ = {
        match_iter->second.pose_.unit_quaternion().slerp(s, match_iter_n->second.pose_.unit_quaternion()),
        match_iter->second.pose_.translation() * (1 - s) + match_iter_n->second.pose_.translation() * s};

    best_match.heading_valid_ = best_iter->second.heading_valid_;
    best_match.status_ = best_iter->second.status_;
    return true;
}

bool DRFrontend::ProcessIMU(ImuMsgPtr imu_msg) {
    dr_->SetImuElements(*imu_msg);
    dr_->ApplyDeadReckoning();

    auto dr_result = dr_->GetDRResult();
    if (dr_result.valid) {
        SE3 dr_pose = DRElementsToSE3(dr_result);
        dr_path_.insert({dr_result.header.stamp.toSec(), dr_pose});
        return true;
    }
    return false;
}

void DRFrontend::ProcessOdom(OdomMsgPtr odom_msg) { dr_->SetOdomElements(*odom_msg); }

void DRFrontend::CollectMessage(std::shared_ptr<Trajectory> t) {
    for (auto &bag_path_pair : t->bag_files) {
        rosbag::Bag bag;
        bag.open(bag_path_pair.second, rosbag::bagmode::Read);
        for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
            auto im = m.instantiate<ImuMsg>();
            if (im != nullptr) {
                t->imu_msgs_.insert({im->header.stamp.toSec(), im});
            }
            auto om = m.instantiate<OdomMsg>();
            if (om != nullptr) {
                t->odom_msgs_.insert({om->header.stamp.toSec(), om});
            }

            auto gm = m.instantiate<GpsMsg>();
            if (gm != nullptr) {
                t->gps_msgs_.insert({gm->header.stamp.toSec(), gm});
            }
        }
    }
    LOG(INFO) << "traj " << t->trajectory_id << " msgs: gps=" << t->gps_msgs_.size() << ", imu=" << t->imu_msgs_.size()
              << ", odom=" << t->odom_msgs_.size();
    for (auto &bag_path_pair : t->bag_files) {
        LOG(INFO) << "include bag: " << bag_path_pair.second;
    }
}

bool DRFrontend::Save() { return true; }

bool DRFrontend::Load() { return true; }

bool DRFrontend::FillTaskInfo(TaskInfo &info) {
    info.length = task_info_.length;
    info.area = task_info_.area;
    info.valid_trajs = task_info_.valid_trajs;
    info.total_keyframes = task_info_.total_keyframes;

    return true;
}

void DRFrontend::SetDebugParams(bool save_pcd) {
    debug_save_pcd_ = save_pcd;
    LOG(INFO) << "save pcd? " << debug_save_pcd_;
}

int DRFrontend::CheckSutengLidarType(const std::string suteng_type) {
    const std::regex check_lidar_type = std::regex(R"(\s*RS(\d+)\s*)", std::regex::icase);
    int suteng_lidartype = 0;
    std::smatch match_result;
    if (std::regex_match(suteng_type, match_result, check_lidar_type)) {
        suteng_lidartype = std::stoi(match_result[(int)match_result.size() - 1]);
    } else {
        LOG(ERROR) << "Can not parse suteng lidar type: " << suteng_type;
    }
    return suteng_lidartype;
}

void DRFrontend::ResetKeyFrameTimeAndPose(const double &scan_time, std::shared_ptr<common::KeyFrame> kf) {
    // LOG(INFO) << "ResetKeyFrameTimeAndPose!";
    if (kf_to_dr_poses_.find(kf) == kf_to_dr_poses_.end()) {
        LOG(WARNING) << "Can not find dr pose for kf!";
        return;
    }

    if (all_gps_path_.find(kf->trajectory_id_) == all_gps_path_.end()) {
        LOG(WARNING) << "Can not find gps_path_ for kf!";
        return;
    }

    auto gps_path = all_gps_path_[kf->trajectory_id_];
    auto pose_buffer = kf_to_dr_poses_[kf];

    kf->timestamp_ = scan_time;
    // find gps pose
    common::GpsPoseStatus gps_pose_status;
    bool has_gps = FindSynGpsPose(scan_time, gps_path, gps_pose_status);
    if (has_gps) {
        kf->gps_status_ = gps_pose_status.status_;
        kf->gps_pose_ = gps_pose_status.pose_;
        kf->heading_valid_ = gps_pose_status.heading_valid_;
    } else {
        // LOG(INFO) << "no gps!";
    }

    std::deque<common::TimedPose> pose_deque;
    for (auto &dr_pose : pose_buffer) {
        pose_deque.push_back(dr_pose);
    }

    auto iter_dr =
        FindClosest<common::TimedPose>(scan_time, pose_deque, [](common::TimedPose dr_pose) { return dr_pose.time; });
    kf->dr_pose_ = iter_dr->pose;

    // LOG(INFO) << "closest dr index in buffer: "<<iter_dr - pose_deque.begin();

    return;
}

}  // namespace mapping::pipeline