//
// Created by gaoxiang on 2020/9/16.
//

#ifndef MAPPING_OPTIMIZATION_PARAMS_H
#define MAPPING_OPTIMIZATION_PARAMS_H

#include "io/yaml_io.h"

namespace mapping::pipeline {

struct OptimizationParams {
    void LoadFromYAML(const io::YAML_IO &yaml) {
        matching_weight = yaml.GetValue<double>("optimization_params", "matching_weight");
        gps_weight = yaml.GetValue<double>("optimization_params", "gps_weight");
        loop_weight = yaml.GetValue<double>("optimization_params", "loop_weight");
        dr_weight = yaml.GetValue<double>("optimization_params", "dr_weight");
        dr_yaw_weight = yaml.GetValue<double>("optimization_params", "dr_yaw_weight");
        matching_chi2_th = yaml.GetValue<double>("optimization_params", "matching_chi2_th");
        dr_chi2_th = yaml.GetValue<double>("optimization_params", "dr_chi2_th");
        gps_chi2_th = yaml.GetValue<double>("optimization_params", "gps_chi2_th");
        loop_chi2_th = yaml.GetValue<double>("optimization_params", "loop_chi2_th");
        gps_height_noise_ratio = yaml.GetValue<double>("optimization_params", "gps_height_noise_ratio");
        loop_type = yaml.GetValue<double>("optimization_params", "loop_type");

        dr_continous_num = yaml.GetValue<double>("optimization_params", "dr_continous_num");
        lidar_continous_num = yaml.GetValue<double>("optimization_params", "lidar_continous_num");
        set_height_zero = yaml.GetValue<bool>("feature_matching_params", "set_height_zero");
        gps_status_set = yaml.GetValue<int>("gps_status");

        with_height = yaml.GetValue<bool>("optimization_params", "with_height");
        with_global_rotation = yaml.GetValue<bool>("optimization_params", "with_global_rotation");

        use_rk_dr = yaml.GetValue<bool>("optimization_params", "use_rk_dr");
        use_rk_matching = yaml.GetValue<bool>("optimization_params", "use_rk_matching");
    }

    double matching_weight{};             // lidar factor 权重
    double gps_weight{};                  // GPS权重
    double loop_weight{};                 // 回环权重
    double dr_weight{};                   // DR 权重
    double dr_yaw_weight{};               // DR YAW权重
    double matching_chi2_th{};            // matching阈值
    double dr_chi2_th{};                  // dr 阈值
    double gps_chi2_th{};                 // gps 阈值
    double loop_chi2_th{};                // loop 阈值
    double gps_height_noise_ratio = 5.0;  // GPS高度值噪声倍数
    int dr_continous_num = 5;             // dr 约束连续帧数
    int lidar_continous_num = 5;          // lidar 约束连续帧数
    bool set_height_zero = false;         // 是否限制高度为零
    int gps_status_set = 3;               // GPS 状态
    bool with_height = true;              // 使用层级和高度约束
    bool with_global_rotation = false;    // 使用全局旋转约束（可能导致问题）
    bool use_rk_matching = true;          // matching中使用robust kernel
    bool use_rk_dr = true;                // dr中使用robust kernel
    int loop_type{};
};

}  // namespace mapping::pipeline
#endif  // MAPPING_OPTIMIZATION_PARAMS_H
