//
// Created by gaoxiang on 2020/8/11.
//
#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "common/common_func.h"
#include "io/file_io.h"
#include "io/xml_io.h"
#include "pipeline/check_in/check_in.h"
#include "pipeline/check_in/file_filter.h"

using namespace mapping::common;
using namespace mapping::io;

namespace mapping::pipeline {

CheckIn::CheckIn(const io::YAML_IO &yaml_file, RunMode run_mode) : PipelineContext(run_mode), yaml_(yaml_file) {
    context_name_ = "CheckIn";
}

CheckIn::~CheckIn() { LOG(INFO) << "check in deconstructed."; }

bool CheckIn::Init() {
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        GenerateGemReport();
    }

    local_data_path_ = yaml_.GetValue<std::string>("data_fetching", "local_data_path");
    return true;
}

bool CheckIn::Start() {
    SetStatus(ContextStatus::WORKING);
    if (!CheckFiles()) {
        LogAndReport("文件不完整");
        SetStatus(ContextStatus::FAILED);
        check_in_falg_ = false;
        return false;
    }

    if (!CheckLaunchFile()) {
        LogAndReport("launch文件有误.");
        SetStatus(ContextStatus::FAILED);
        check_in_falg_ = false;
        return false;
    }

    // save to yaml
    if (car_type_ == CarType::WXX) {
        yaml_.SetValue<std::string>("car_type", "wxx");
        LogAndReport("车辆类型：蜗小白/蜗必达");
    } else if (car_type_ == CarType::LADS) {
        bool if_allow_lads_params = yaml_.GetValue<bool>("allow_default_lads_params");
        if (!if_allow_lads_params || run_mode_ == RunMode::GEM_EXECUTABLE) {
            yaml_.SetValue<std::string>("car_type", "lads");
        } else {
            ResetYamlToLadsParams();
        }
        LogAndReport("车辆类型：LADS");
    }

    SaveParams();
    LogAndReport("准入通过");
    SetStatus(ContextStatus::SUCCEED);

    return true;
}

void CheckIn::ResetYamlToLadsParams() {
    // update local_data_path_ / local_db_path / map_name / data_url / if_merge_maps / if_map_updating /
    // origin_map_db_path
    std::string local_data_path = yaml_.GetValue<std::string>("data_fetching", "local_data_path");
    std::string local_db_path = yaml_.GetValue<std::string>("local_db_path");
    std::string map_name = yaml_.GetValue<std::string>("map_name");
    std::string data_url = yaml_.GetValue<std::string>("data_fetching", "data_url");
    bool if_merge_maps = yaml_.GetValue<bool>("if_merge_maps");
    bool if_map_updating = yaml_.GetValue<bool>("if_map_updating");
    std::string origin_map_db_path = yaml_.GetValue<std::string>("origin_map_db_path");

    //替换原有的yaml
    std::string current_config_file;
    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        current_config_file = "/home/idriver/work/share/config/" + map_name + ".yaml";
    } else {
        std::shared_ptr<io::YAML_IO> lads_yaml;
        lads_yaml.reset(new io::YAML_IO("./config/mapping_lads.yaml"));
        current_config_file = "./config/" + map_name + ".yaml";
        lads_yaml->Save(current_config_file);
    }

    io::YAML_IO new_lads_yaml(current_config_file);
    new_lads_yaml.SetValue("data_fetching", "local_data_path", local_data_path);
    new_lads_yaml.SetValue("local_db_path", local_db_path);
    new_lads_yaml.SetValue("map_name", map_name);
    new_lads_yaml.SetValue("data_fetching", "data_url", data_url);
    new_lads_yaml.SetValue("if_merge_maps", if_merge_maps);
    new_lads_yaml.SetValue("if_map_updating", if_map_updating);
    new_lads_yaml.SetValue("origin_map_db_path", origin_map_db_path);
    new_lads_yaml.Save();

    yaml_ = new_lads_yaml;

    return;
}

bool CheckIn::CheckFiles() {
    FilesFilter files_filter(local_data_path_);
    if (io::PathExists(local_data_path_ + "/ConvertData")) {
        // 删除旧的convert data
        LogAndReport("删除旧的转换数据: " + local_data_path_ + "/ConvertData");
        boost::filesystem::remove_all(local_data_path_ + "/ConvertData");
    }

    files_filter.FilterAllFolderUnderParent();
    origin_files_ = files_filter.GetFilteredFiles();

    // 确定车辆类型
    if (IsWXX()) {
        car_type_ = common::CarType::WXX;
        lidar_param_.lidar_type = 16;
    } else if (IsLADS()) {
        car_type_ = common::CarType::LADS;

    } else {
        LogAndReport("未能识别车辆类型，请检查数据包格式是否正确！");
        return false;
    }

    /// 判断数据包
    if (origin_files_.bag_files.size() < 1) {
        LogAndReport("未找到ROS数据包");
        failed_reason_ = "未找到ROS数据包";
        return false;
    } else {
        int d_bag_num = 0;
        int z_bag_num = 0;
        for (auto &bag_file : origin_files_.bag_files) {
            std::vector<std::string> re;
            SplitString(bag_file, '/', re);
            std::string bag_name = re.at(re.size() - 1);
            if ('d' == bag_name.at(0)) {
                d_bag_num++;
            }
            if ('z' == bag_name.at(0)) {
                z_bag_num++;
            }
        }

        if (d_bag_num < 1) {
            LogAndReport("建图包数量不足：" + std::to_string(d_bag_num));
            failed_reason_ = "建图包数量不足：";
            return false;
        }

        if (car_type_ == common::CarType::WXX && z_bag_num < 1) {
            LogAndReport("未找到蜗小白地图的验证包");
            failed_reason_ = "未找到蜗小白地图的验证包";
            return false;
        }

        LogAndReport("建图包数量：" + std::to_string(d_bag_num) + "，验证包数量：" + std::to_string(z_bag_num));
    }

    return true;
}

bool CheckIn::IsWXX() {
    bool calibration_exist = io::PathExists(local_data_path_ + "/calibration.launch");
    bool vehicle_param_exist = io::PathExists(local_data_path_ + "/vehicleparams.launch");

    return calibration_exist && vehicle_param_exist;
}

bool CheckIn::IsLADS() {
    bool localization_map_collection_yaml_exist =
        io::PathExists(local_data_path_ + "/localization_map_collection.yaml");
    return localization_map_collection_yaml_exist;
}

bool CheckIn::CheckLaunchFile() {
    if (car_type_ == CarType::WXX) {
        XML_IO xml_parser(origin_files_.calib_params);
        std::string xml_name = "calibration.launch";
        if (xml_parser.ParseXml(xml_name) != 1) {
            LogAndReport("解析calibration.launch出错");
            failed_reason_ = "解析calibration.launch出错";
            return false;
        }

        lidar_param_ = xml_parser.GetParamForMapping();
        lidar_param_.lidar_factory = "velodyne";
        lidar_param_.lidar_type = 16;

    } else if (car_type_ == CarType::LADS) {
        io::YAML_IO yaml(local_data_path_ + "localization_map_collection.yaml");
        auto lidar_launch = yaml.GetValue<std::string>("lidar_launch");
        auto sensor_calibration_conf = yaml.GetValue<std::string>("sensor_calibration_conf");
        auto localization_conf = yaml.GetValue<std::string>("localization_conf");

        std::vector<std::string> conf;
        conf.push_back(local_data_path_ + localization_conf);
        conf.push_back(local_data_path_ + sensor_calibration_conf);

        std::string lidar_factory = lidar_launch.substr(0, lidar_launch.find('.'));
        std::string is_irad = lidar_launch.substr(lidar_launch.find('.') + 1, lidar_launch.length() - 1);
        if (!CheckLidarManufacturer(lidar_factory)) {
            LogAndReport("未支持的雷达的类型" + lidar_factory);
            return false;
        } else {
            LogAndReport("雷达生产厂商: " + lidar_factory);
        }

        LOG(INFO) << "local_data_path_ + lidar_launch: " << local_data_path_ + lidar_launch;
        XML_IO xml_parser(conf, local_data_path_ + lidar_launch, lidar_factory);
        auto re = xml_parser.ParseXml();
        if (re != 0) {
            LOG(ERROR) << "ERROR : xml parse failed!!!  " << re;
            report_ += "check_in error: xml parse failed (" + std::to_string(re) + ").\n";
            failed_reason_ = "解析XML失败";
            return false;
        }

        lidar_param_ = xml_parser.GetParamForMapping();
        lidar_param_.lidar_factory = lidar_factory;
        lidar_param_.is_irad = (is_irad == "irad" ? true : false);

    } else {
        LogAndReport("车辆类型有误");
        failed_reason_ = "车辆类型有误";
        return false;
    }
    return true;
}

bool CheckIn::CheckLidarManufacturer(const std::string &lidar_name) {
    const std::vector<std::string> supported_lidars{"suteng", "hesai", "velodyne"};

    return std::find(supported_lidars.begin(), supported_lidars.end(), lidar_name) != supported_lidars.end();
}

void CheckIn::SaveParams() {
    LOG(INFO) << "saving params ... ";
    // 将参数保存至yaml
    // lidar 和 gps 参数
    yaml_.SetValue<double>("gps_params", "ant_x", lidar_param_.antenna_x);
    yaml_.SetValue<double>("gps_params", "ant_y", lidar_param_.antenna_y);
    yaml_.SetValue<double>("gps_params", "ant_angle", lidar_param_.antenna_angle);
    yaml_.SetValue<double>("velodyne_calib_param", "xoffset", lidar_param_.perception_lidar_x_offset_top_center);
    yaml_.SetValue<double>("velodyne_calib_param", "yoffset", lidar_param_.perception_lidar_y_offset_top_center);
    yaml_.SetValue<double>("velodyne_calib_param", "zoffset", lidar_param_.perception_lidar_z_offset_top_center);
    yaml_.SetValue<double>("velodyne_calib_param", "roll", lidar_param_.perception_lidar_roll_top_center);
    yaml_.SetValue<double>("velodyne_calib_param", "pitch", lidar_param_.perception_lidar_pitch_top_center);
    yaml_.SetValue<double>("velodyne_calib_param", "yaw", lidar_param_.perception_lidar_yaw_top_center);
    yaml_.SetValue<int>("velodyne_calib_param", "pre_rot_axis_0", lidar_param_.pre_rot_axis_0);
    yaml_.SetValue<int>("velodyne_calib_param", "pre_rot_axis_1", lidar_param_.pre_rot_axis_1);
    yaml_.SetValue<int>("velodyne_calib_param", "pre_rot_axis_2", lidar_param_.pre_rot_axis_2);
    yaml_.SetValue<double>("velodyne_calib_param", "pre_rot_degree_0", lidar_param_.pre_rot_degree_0);
    yaml_.SetValue<double>("velodyne_calib_param", "pre_rot_degree_1", lidar_param_.pre_rot_degree_1);
    yaml_.SetValue<double>("velodyne_calib_param", "pre_rot_degree_2", lidar_param_.pre_rot_degree_2);
    yaml_.SetValue<bool>("velodyne_calib_param", "is_irad", lidar_param_.is_irad);

    // 车辆相关参数
    if (car_type_ == common::CarType::WXX) {
        yaml_.SetValue<double>("dr_params", "static_gyro_var", 0.02);
        yaml_.SetValue<double>("dr_params", "static_acc_var", 0.005);
        int gmm_map = yaml_.GetValue<int>("simulation_param", "gmm_map");
        if (0 != gmm_map) {
            yaml_.SetValue<int>("simulation_param", "gmm_map", 1);
        }
    } else if (car_type_ == common::CarType::LADS) {
        yaml_.SetValue<double>("dr_params", "static_gyro_var", lidar_param_.static_gyro_var);
        yaml_.SetValue<double>("dr_params", "static_acc_var", lidar_param_.static_acc_var);
        int gmm_map = yaml_.GetValue<int>("simulation_param", "gmm_map");
        if (0 != gmm_map) {
            yaml_.SetValue<int>("simulation_param", "gmm_map", 3);
        }
    }

    yaml_.SetValue<double>("dr_params", "ratio_left", lidar_param_.odom_ratio_left);
    yaml_.SetValue<double>("dr_params", "ratio_right", lidar_param_.odom_ratio_right);
    yaml_.SetValue<double>("msf_params", "odom_ratio_left", lidar_param_.odom_ratio_left);
    yaml_.SetValue<double>("msf_params", "odom_ratio_right", lidar_param_.odom_ratio_right);
    yaml_.SetValue<int>("velodyne_calib_param", "type", lidar_param_.lidar_type);
    yaml_.SetValue<std::string>("velodyne_calib_param", "factory", lidar_param_.lidar_factory);

    if (16 == lidar_param_.lidar_type && lidar_param_.lidar_factory == "velodyne") {
        yaml_.SetValue<std::string>("lidar_frontend_params", "matching_type", "feature_matching");
    } else if (32 == lidar_param_.lidar_type && lidar_param_.lidar_factory == "velodyne") {
        yaml_.SetValue<std::string>("lidar_frontend_params", "matching_type", "ndt_matching");
    } else if (64 == lidar_param_.lidar_type && lidar_param_.lidar_factory == "velodyne") {
        yaml_.SetValue<std::string>("lidar_frontend_params", "matching_type", "ndt_matching");
    } else if (16 == lidar_param_.lidar_type && lidar_param_.lidar_factory == "suteng") {
        yaml_.SetValue<std::string>("lidar_frontend_params", "matching_type", "feature_matching");
    } else if (64 == lidar_param_.lidar_type && lidar_param_.lidar_factory == "hesai") {
        yaml_.SetValue<std::string>("lidar_frontend_params", "matching_type", "ndt_matching");
    } else {
        yaml_.SetValue<std::string>("lidar_frontend_params", "matching_type", "ndt_matching");
    }

    if (run_mode_ == RunMode::GEM_EXECUTABLE) {
        yaml_.SetValue("task_info", "failed_reason", failed_reason_);
        SaveGemReport(local_data_path_ + gem_report_name_);
    }

    yaml_.Save();
}

bool CheckIn::FillTaskInfo(TaskInfo &info) {
    info.failed_reason = failed_reason_;
    info.checkin_passed = check_in_falg_;
    return true;
}

bool CheckIn::Save() { return true; }

bool CheckIn::Load() { return true; }

}  // namespace mapping::pipeline
