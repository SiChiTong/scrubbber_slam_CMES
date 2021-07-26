//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_PREPROCESSING_H
#define MAPPING_PREPROCESSING_H

#include <utility>

#include "common/car_type.h"
#include "common/gps_status_def.h"
#include "common/message_def.h"
#include "common/origin_files.h"
#include "common/origin_point_info.h"
#include "common/trajectory.h"
#include "common/vehicle_calib_param.h"
#include "io/yaml_io.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

struct BagGpsInfo {
    BagGpsInfo(common::BagGNSSStatusType gps_stat, std::string path, GpsMsg msg)
        : gps_status(gps_stat), bag_path(std::move(path)), stable_gps_msg(msg) {}

    common::BagGNSSStatusType gps_status = common::BagGNSSStatusType::BAG_GNSS_NOT_EXIST;
    std::string bag_path;
    GpsMsg stable_gps_msg;
};

/**
 * 预处理：收集各包数据，格式转换，填写地图GPS原点信息
 * NOTE 这一步不会生成关键帧，关键帧在Dr_frontend中生成
 */
class Preprocessing : public PipelineContext {
   public:
    /// 给定配置文件
    Preprocessing(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE);
    ~Preprocessing() override;

    /// Context 接口
    /// 初始化，成功返回 true
    virtual bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    virtual bool Start() override;

    /// 缓存中间结果
    virtual bool Save() override;

    virtual bool Load() override;

    /// 填写任务信息
    virtual bool FillTaskInfo(TaskInfo &info) override;

   private:
    /// 包统一格式转换
    void ConvertBags();

    /// 收集各包GPS状态
    void CollectGpsStatus();

    /// 分析整体建图GPS状态
    bool AnalyzeGpsStatus();

    /// 填写地图原点信息
    void FillOriginPoint();

    /// 将建图文件夹中参数保存至配置文件
    void SaveParams();

    /// 读取雷达参数
    void LoadLidarParams();

    /// 根据包名分析包间关系
    bool ParseBagsByName();

    /// 根据名称查找对应的trajectory
    std::shared_ptr<common::Trajectory> FindStartTrajectory(const std::string &bag_name);

    /// 根据是否分包及当前traj id查找对应trajectory
    std::shared_ptr<common::Trajectory> FindStartTrajectory(const bool &is_part_bag, IdType traj_id);

    std::string report_;
    io::YAML_IO yaml_file_;
    TaskInfo task_info_;

    std::string local_data_path_;
    std::string origin_map_db_path_;
    bool if_updating_maps_ = false;                // 是否为更新任务
    bool if_merge_maps_ = false;                   // 是否为合并任务
    common::CarType car_type_;                     // 车辆类型
    common::OriginFiles origin_files_;             // 原始数据文件
    common::OriginPointInformation origin_info_;   // 地图原点信息
    common::VehicleCalibrationParam lidar_param_;  // 雷达参数
    std::vector<BagGpsInfo> gps_status_;           // 各包GPS状态信息
    std::string scene_type_;                       // 场景类型，可能为indoor/outdoor/half indoor
    common::BagGNSSStatusType max_gps_status_ = common::BagGNSSStatusType::BAG_GNSS_NOT_EXIST;  // 最好的GNSS状态
    /// DR轨迹
    std::map<IdType, std::shared_ptr<common::Trajectory>> trajectory_;
};

}  // namespace mapping::pipeline

#endif  // MAPPING_PREPROCESSING_H
