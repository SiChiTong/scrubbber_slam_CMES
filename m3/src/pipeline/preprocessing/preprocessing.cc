//
// Created by gaoxiang on 2020/8/11.
//

#include "pipeline/preprocessing/preprocessing.h"
#include "common/topics_def.h"
#include "core/coordinate_transform/gps_trans.h"
#include "io/file_io.h"
#include "io/xml_io.h"
#include "pipeline/check_in/file_filter.h"
#include "pipeline/preprocessing/bag_convert.h"

#include <glog/logging.h>
#include <rosbag/view.h>

namespace mapping::pipeline {

using namespace mapping::common;

std::map<std::string, std::string> scene_type_mapping = {
    {"Indoor", "室内"}, {"Half Indoor", "Half Indoor"}, {"Outdoor", "室外"}};

Preprocessing::Preprocessing(const io::YAML_IO &yaml_file, RunMode run_mode) : yaml_file_(yaml_file) {
    context_name_ = "Preprocessing";
}

Preprocessing::~Preprocessing() { LOG(INFO) << "Preprocessing deconstructed."; }

bool Preprocessing::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }
    local_data_path_ = yaml_file_.GetValue<std::string>("data_fetching", "local_data_path");
    origin_map_db_path_ = yaml_file_.GetValue<std::string>("origin_map_db_path");
    if_updating_maps_ = yaml_file_.GetValue<bool>("if_map_updating");
    if_merge_maps_ = yaml_file_.GetValue<bool>("if_merge_maps");

    if (io::PathExists(local_data_path_ + "/ConvertData")) {
        /// 删除已经转换过的文件
        std::string cmd = std::string("rm -rf ") + local_data_path_ + "/ConvertData/*";
        LOG(INFO) << cmd;
        system(cmd.c_str());
    } else {
        /// 创建文件转换目录
        std::string cmd = std::string("mkdir -p ") + local_data_path_ + "/ConvertData";
        LOG(INFO) << cmd;
        system(cmd.c_str());
    }

    /// 查找原始数据
    FilesFilter files_filter(local_data_path_);
    files_filter.FilterAllFolderUnderParent();
    origin_files_ = files_filter.GetFilteredFiles();

    /// load lidar params and car type
    std::string car_type = yaml_file_.GetValue<std::string>("car_type");
    if (car_type == "wxx" || car_type == "wxb") {
        car_type_ = common::CarType::WXX;
    } else if (car_type == "lads") {
        car_type_ = common::CarType::LADS;

        io::YAML_IO yaml(local_data_path_ + "localization_map_collection.yaml");
        std::string lidar_launch = yaml.GetValue<std::string>("lidar_launch");
        std::string lidar_factory = yaml_file_.GetValue<std::string>("velodyne_calib_param", "factory");

        // TODO: support other lidars
        io::XML_IO xml_parser(origin_files_.conf_params, lidar_launch, lidar_factory);
        int xml_ret = xml_parser.ParseXml();
        if (xml_ret != 0) {  // xml_ret != 1
            LOG(ERROR) << "parse xml failed.";
            return false;
        }

        lidar_param_ = xml_parser.GetParamForMapping();
    } else {
        LogAndReport("未知的车辆类型: " + car_type);
        return false;
    }

    LoadLidarParams();

    return true;
}

bool Preprocessing::Start() {
    SetStatus(ContextStatus::WORKING);

    // 将不同种类地图包转换为统一格式
    ConvertBags();

    // 计算各包的GPS稳定性
    CollectGpsStatus();

    /// 分析整体建图GPS状态
    AnalyzeGpsStatus();

    /// 填写原点信息
    FillOriginPoint();

    // 检查自动分包，创建轨迹
    LogAndReport("解析数据包名称中");
    if (ParseBagsByName() == false) {
        SetStatus(ContextStatus::FAILED);
        return false;
    }

    /// 保存参数至yaml
    SaveParams();

    SetStatus(ContextStatus::SUCCEED);
    return true;
}

void Preprocessing::LoadLidarParams() { io::LoadVehicleParamsFromYAML(yaml_file_, lidar_param_); }

void Preprocessing::ConvertBags() {
    std::vector<std::thread> convert_ths;

    for (auto &bag_file : origin_files_.bag_files) {
        // convert_ths.emplace_back([this, &bag_file]() {
        //     BagConvert bc;
        //     std::string bag_file_name = bag_file.substr(bag_file.find_last_of('/'), std::string::npos);
        //     bag_file_name = bag_file_name.substr(0, bag_file_name.size() - 4);  // 去掉.bag
        //     std::string output_file = local_data_path_ + "/ConvertData/" + bag_file_name + "_V.bag";
        //     origin_files_.converted_bag_files.push_back(output_file);
        //     bc.Run(bag_file, output_file);
        // });
        BagConvert bc;
        std::string bag_file_name = bag_file.substr(bag_file.find_last_of('/'), std::string::npos);
        bag_file_name = bag_file_name.substr(0, bag_file_name.size() - 4);  // 去掉.bag
        std::string output_file = local_data_path_ + "/ConvertData/" + bag_file_name + "_V.bag";
        origin_files_.converted_bag_files.push_back(output_file);
        bc.Run(bag_file, output_file);
    }

    // for (auto &th : convert_ths) {
    //     th.join();
    // }

    LOG(INFO) << "Finish Convert Bags!";

    convert_ths.clear();

    return;
}

/**
 * 1. GPS pose and angle are valid at the same time, return 3,
 * 2. GPS pose and angle are valid not at the same time, return 2,
 * 3. GPS pose is good, return 1;
 * 4. GPS pose is not precision and no valid gps(not 1 and 2), return 0,
 * 5. GPS no pose, return -1
 */
void Preprocessing::CollectGpsStatus() {
    for (auto &bag_file : origin_files_.converted_bag_files) {
        rosbag::Bag bag;
        bag.open(bag_file, rosbag::bagmode::Read);

        bool flag_status = true;
        int stop_0 = 1;
        bool stop_1 = 0;
        bool stop_pose = 1;
        bool stop_angle = 1;
        int stable_count_0 = 0;
        int stable_count_1 = 0;
        int stable_count_pose = 0;
        int stable_count_angle = 0;

        BagGNSSStatusType res = BagGNSSStatusType::BAG_GNSS_NOT_EXIST;
        const int stable_gps_time = 10;
        std::vector<GpsMsg::Ptr> gps_msgs;

        for (const auto &m : rosbag::View(bag)) {
            if (m.getTopic() != common::topics::tp_gps) {
                continue;
            }
            auto msg = m.instantiate<GpsMsg>();
            if (msg == nullptr) {
                continue;
            }
            gps_msgs.push_back(msg);
        }

        GpsMsg converged_gps;
        for (size_t i = 0; i < gps_msgs.size(); ++i) {
            auto msg = gps_msgs[i];
            if (msg->is_heading_valid == 1 && msg->status == 4) {
                flag_status = true;
            } else {
                flag_status = false;
            }

            if (flag_status) {
                stable_count_1++;
            } else {
                stable_count_1 = 0;
            }

            if (stable_count_1 >= stable_gps_time && 0 == stop_1) {
                converged_gps = *gps_msgs.at(i - stable_gps_time / 2);
                stop_1 = 1;
                break;
            }

            if (msg->status == 4) {
                flag_status = true;
            } else {
                flag_status = false;
            }

            if (flag_status) {
                stable_count_pose++;
            } else {
                stable_count_pose = 0;
            }

            if (stable_count_pose >= stable_gps_time && 1 == stop_pose) {
                if (stop_1 == 0) {
                    converged_gps = *gps_msgs.at(i - stable_gps_time / 2);
                }
                stop_pose = 0;
            }

            if (msg->is_heading_valid == 1) {
                flag_status = true;
            } else {
                flag_status = false;
            }

            if (flag_status) {
                stable_count_angle++;
            } else {
                stable_count_angle = 0;
            }

            if (stable_count_angle >= stable_gps_time && 1 == stop_angle) {
                stop_angle = 0;
            }

            if (msg->status != 0) {
                flag_status = true;
            } else {
                flag_status = false;
            }

            if (flag_status) {
                stable_count_0++;
            } else {
                stable_count_0 = 0;
            }

            if (stable_count_0 >= stable_gps_time && 1 == stop_0) {
                if (stop_1 == 0 && stop_pose == 1) {
                    converged_gps = *gps_msgs.at(i - stable_gps_time / 2);
                }
                stop_0 = 0;
            }
        }

        if (1 == stop_1) {
            res = BagGNSSStatusType::BAG_GNSS_POS_HEAD_VALID_SYNC;
        } else if (0 == stop_pose && 0 == stop_angle) {
            res = BagGNSSStatusType::BAG_GNSS_POS_HEAD_VALID_UNSYNC;
        } else if (0 == stop_pose) {
            res = BagGNSSStatusType::BAG_GNSS_EXIST;
        } else if (0 == stop_0) {
            res = BagGNSSStatusType::BAG_GNSS_EXIST;
        } else {
            res = BagGNSSStatusType::BAG_GNSS_NOT_EXIST;
        }

        gps_status_.push_back({res, bag_file, converged_gps});
    }

    assert(gps_status_.size() == origin_files_.bag_files.size());
    for (auto gp : gps_status_) {
        LOG(INFO) << "bag " << gp.bag_path << " gps st: " << int(gp.gps_status);
    }
}

bool Preprocessing::AnalyzeGpsStatus() {
    int d_bag_num = 0;
    int d_bag_with_good_gps = 0;
    int z_bag_num = 0;
    int z_bag_with_good_gps = 0;
    int d_bag_with_bad_gps = 0;
    int z_bag_with_bad_gps = 0;
    int no_gps_bag_num = 0;

    for (auto &gps_status : gps_status_) {
        std::string full_bag_path = gps_status.bag_path;
        std::string bag_name = full_bag_path.substr(full_bag_path.find_last_of('/') + 1, full_bag_path.size());
        auto gs = gps_status.gps_status;

        if ('d' == bag_name.at(0)) {
            d_bag_num++;
            if (int(gs) > 0) {
                d_bag_with_good_gps++;
            } else if (int(gs) == 0) {
                d_bag_with_bad_gps++;
            } else if (int(gs) <= -1) {
                no_gps_bag_num++;
            }

            if (gs > max_gps_status_) {
                max_gps_status_ = gs;
            }

        } else if ('z' == bag_name.at(0)) {
            z_bag_num++;
            if (int(gs) > 0) {
                z_bag_with_good_gps++;
            } else if (int(gs) == 0) {
                z_bag_with_bad_gps++;
            } else if (int(gs) <= -1) {
                no_gps_bag_num++;
            }
        }
    }

    if (no_gps_bag_num > 0) {
        /// 至少一个包没有GPS数据
        scene_type_ = "Indoor";
    } else if ((d_bag_with_bad_gps > 0 || z_bag_with_bad_gps > 0) && (d_bag_with_good_gps == 0)) {
        scene_type_ = "Half Indoor";
    } else if ((d_bag_with_good_gps == 0 && d_bag_with_bad_gps == 0) ||
               (z_bag_with_bad_gps == 0 && z_bag_with_good_gps == 0)) {
        scene_type_ = "Indoor";
    } else if (((d_bag_with_good_gps + d_bag_with_bad_gps == d_bag_num) && (d_bag_with_good_gps != 0)) ||
               ((z_bag_with_good_gps + z_bag_with_bad_gps == z_bag_num) && (z_bag_with_good_gps != 0))) {
        scene_type_ = "Outdoor";
    } else if (d_bag_with_good_gps == d_bag_num || (z_bag_with_good_gps == z_bag_num)) {
        scene_type_ = "Outdoor";
    } else {
        scene_type_ = "Half Indoor";
    }

    yaml_file_.SetValue<int>("gps_status", int(max_gps_status_));
    LogAndReport("最佳GPS状态：" + std::to_string(int(max_gps_status_)));

    /// 报告GPS可能存在的问题
    if (d_bag_with_bad_gps > 0) {
        LogAndReport("\\textbf{警告}: 有" + std::to_string(d_bag_with_bad_gps) + "个建图包GPS信号不佳：");
    }

    if (z_bag_with_bad_gps > 0) {
        LogAndReport("\\textbf{警告}: 有" + std::to_string(z_bag_with_bad_gps) + "个验证包GPS信号不佳：");
    }

    return true;
}

void Preprocessing::FillOriginPoint() {
    bool origin_get = false;
    // 如果是地图新增或者更新，需要在原先的地图数据（map.db）中获取地图原点信息
    if (if_merge_maps_ || if_updating_maps_) {
        report_ += "Preprocessing: the map is an merge or update.\n";
        origin_map_db_path_ = origin_map_db_path_ + "/map.db";
        if (io::LoadOriginInfoByDB(origin_map_db_path_, origin_info_)) origin_get = true;
    }
    if (max_gps_status_ == BagGNSSStatusType::BAG_GNSS_NOT_EXIST && !origin_get) {
        report_ += "Preprocessing: the map has no gps signal.\n";
        origin_info_.map_origin_x = 667220.60791066091;
        origin_info_.map_origin_y = 3523126.7262191805;
        origin_info_.map_origin_z = 0;
        origin_info_.map_origin_zone = 50;
    } else if (int(max_gps_status_) >= 0 && !origin_get) {
        // find stable gps msg
        bool origin_set = false;
        for (auto &gs : gps_status_) {
            if (int(gs.gps_status) > -1) {
                // TODO GPS坐标变换
                core::GpsTransform gt(lidar_param_.antenna_x, lidar_param_.antenna_y, lidar_param_.antenna_angle);
                origin_info_ = gt.GpsToOriginInfo(gs.stable_gps_msg);
                origin_set = true;
                break;
            }
        }

        if (!origin_set) {
            report_ += "Preprocessing: origin point is not set.\n";
            origin_info_.map_origin_x = 443684.11088364013;
            origin_info_.map_origin_y = 4436651.256764573;
            origin_info_.map_origin_z = 42.694816589355469;
            origin_info_.map_origin_zone = 50;
        }
    }

    LogAndReport("地图原点：" + std::to_string(origin_info_.map_origin_x) + ", " +
                 std::to_string(origin_info_.map_origin_y) + ", " + std::to_string(origin_info_.map_origin_z));
}

void Preprocessing::SaveParams() {
    LOG(INFO) << "saving params ... ";
    // 将参数保存至yaml
    // 原点参数
    yaml_file_.SetValue<double>("map_origin_param", "map_origin_x", origin_info_.map_origin_x);
    yaml_file_.SetValue<double>("map_origin_param", "map_origin_y", origin_info_.map_origin_y);
    yaml_file_.SetValue<double>("map_origin_param", "map_origin_z", origin_info_.map_origin_z);
    yaml_file_.SetValue<int>("map_origin_param", "map_origin_zone", origin_info_.map_origin_zone);
    yaml_file_.SetValue<bool>("map_origin_param", "is_southern", origin_info_.is_southern);
    std::string map_name = yaml_file_.GetValue<std::string>("map_name");
    yaml_file_.SetValue<std::string>("own_data_path", "/home/idriver/work/own/" + map_name + "/");
    task_info_.valid_bags = origin_files_.bag_files.size();
    yaml_file_.SetValue("task_info", "valid_bags", task_info_.valid_bags);
    yaml_file_.SetValue("task_info", "scene_type", scene_type_mapping[scene_type_]);
    yaml_file_.Save();

    // 记录trajectory和包的关系
    io::SaveTrajectoryBagName(local_data_path_ + "trajectory_info.txt", trajectory_);
    LogAndReport("预处理完成");

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        SaveGemReport(local_data_path_ + gem_report_name_);
    }
}

bool Preprocessing::FillTaskInfo(TaskInfo &info) {
    info.valid_bags = task_info_.valid_bags;
    return true;
}

bool Preprocessing::Save() { return true; }

bool Preprocessing::Load() { return true; }

bool Preprocessing::ParseBagsByName() {
    /// 匹配包名
    bool has_success = false;
    IdType traj_id = 0;
    std::map<double, io::FileNameInfo> time_to_bag_;

    for (auto &bag_file : origin_files_.converted_bag_files) {
        LOG(INFO) << bag_file;
        auto info = io::ParseBagName(bag_file);
        double bagtime = io::GetBagFileTime(bag_file);

        if (bagtime < 0.0) {
            LogAndReport("无法获取包时间： " + bag_file);
            continue;
        }

        if (!info.parse_success) {
            LogAndReport("无法匹配包名：" + bag_file);
            continue;
        }

        time_to_bag_.insert(std::make_pair(bagtime, info));
        has_success = true;
    }

    for (auto &info : time_to_bag_) {
        bool is_z_bag = info.second.bag_name[0] == 'z';
        bool is_part_bag = info.second.has_part;
        int part_n = info.second.part_num;  // part_num 为空时，默认为0

        if (is_part_bag) {
            part_n += 1;
        }

        LOG(INFO) << std::setprecision(16) << "bag info: " << info.first << " " << part_n << " " << (int)is_part_bag;

        auto traj = FindStartTrajectory(is_part_bag, traj_id);
        if (traj == nullptr) {
            traj = std::make_shared<Trajectory>(traj_id, info.second.bag_name);
            traj->bag_files.insert({part_n, info.second.bag_file_path});
            traj->is_mapping_bag_ = !is_z_bag;
            trajectory_.insert({traj_id, traj});
            ++traj_id;
        } else {
            traj->bag_files.insert({part_n, info.second.bag_file_path});
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

std::shared_ptr<Trajectory> Preprocessing::FindStartTrajectory(const std::string &bag_name) {
    for (auto tp : trajectory_) {
        if (tp.second->bag_name == bag_name) {
            return tp.second;
        }
    }
    return nullptr;
}

std::shared_ptr<Trajectory> Preprocessing::FindStartTrajectory(const bool &is_part_bag, IdType traj_id) {
    for (auto tp : trajectory_) {
        if (is_part_bag && tp.first == traj_id - 1) {
            return tp.second;
        }
    }
    return nullptr;
}

}  // namespace mapping::pipeline
