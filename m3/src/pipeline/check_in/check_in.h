//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_CHECK_IN_H
#define MAPPING_CHECK_IN_H

#include "common/car_type.h"
#include "common/vehicle_calib_param.h"
#include "io/yaml_io.h"
#include "pipeline/check_in/file_filter.h"
#include "pipeline/pipeline_context.h"

namespace mapping::pipeline {

/**
 * 准入：检查文件的存在性与完整性
 *
 * WXB：必须有calibration.launch (xml, 传感器外参数）; vehicleparams.launch（xml，左右轮速）;
 *
 * 低速类建图必须有d和z两种数据包
 * 准入会检查这些文件是否存在，格式是否完整
 *
 * 对于LADS车辆，有一个配置文件为localization_map_collection.yaml，其中 lidar_launch 指明了雷达的类型与参数文件位置
 */
class CheckIn : public PipelineContext {
   public:
    /// 给定配置文件
    explicit CheckIn(const io::YAML_IO &yaml_file, RunMode run_mode = RunMode::PIPELINE);
    ~CheckIn() override;

    /// Context 接口
    /// 初始化，成功返回 true
    bool Init() override;

    /// 开始处理，完成后返回true，无论结果是否成功
    /// 要求完成后更新context status状态，以便 engine 确定它的完成情况
    /// NOTE: 模块内容应阻塞此Start函数，Start函数返回表明已执行完毕
    bool Start() override;

    /// 缓存中间结果
    bool Save() override;

    bool Load() override;

    /// 填写任务信息
    bool FillTaskInfo(TaskInfo &info) override;

   private:
    /// 检查文件存在性
    bool CheckFiles();

    /// 检查launch是否合法
    bool CheckLaunchFile();

    /// 判断是否蜗小白
    bool IsWXX();

    /// 判断是否为LADS采图
    bool IsLADS();

    /// 检查雷达的生产厂家
    bool CheckLidarManufacturer(const std::string &lidar_name);

    /// 将建图文件夹中参数保存至配置文件
    void SaveParams();

    void ResetYamlToLadsParams();

    std::string local_data_path_;
    std::string failed_reason_;
    bool check_in_falg_ = true;

    common::OriginFiles origin_files_;
    common::CarType car_type_ = common::CarType::WXX;
    common::VehicleCalibrationParam lidar_param_;  // 雷达参数

    io::YAML_IO yaml_;
};

}  // namespace mapping::pipeline

#endif  // MAPPING_CHECK_IN_H
