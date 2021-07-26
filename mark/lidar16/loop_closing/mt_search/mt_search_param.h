//
// Created by gaoxiang on 2020/9/22.
//

#ifndef MAPPING_MT_SEARCH_PARAM_H
#define MAPPING_MT_SEARCH_PARAM_H

#include "common/yaml_io.h"

namespace scrubber_slam { namespace lidar16 {

struct MTSearchParams {
    void LoadFromYAML(const scrubber_slam::YAML_IO &yaml_file) {
        closest_id_th_ = yaml_file.GetValue<int>("loop_closure_params", "closest_id_th");
        min_id_interval_ = yaml_file.GetValue<int>("loop_closure_params", "min_id_interval");
        linear_search_meter = yaml_file.GetValue<double>("loop_closure_params", "linear_search_meter");
        angular_search_deg = yaml_file.GetValue<double>("loop_closure_params", "angular_search_deg");
        lidar_max_range = yaml_file.GetValue<double>("loop_closure_params", "lidar_max_range");
        multi_matching_min_score = yaml_file.GetValue<double>("loop_closure_params", "multi_matching_min_score");
        ndt_matching_min_proba = yaml_file.GetValue<double>("loop_closure_params", "ndt_matching_min_proba");
        ground_height = yaml_file.GetValue<double>("loop_closure_params", "ground_height");
        num_threads = yaml_file.GetValue<int>("loop_closure_params", "num_threads");
    }

    int closest_id_th_ = 150;          // id 相近的不搜索
    int min_id_interval_ = 10;         // 连续id不检索
    float linear_search_meter{};       // 多分辨率地图搜索距离
    float angular_search_deg{};        // 多分辨率地图搜索角度
    float lidar_max_range{};           // 回环检测时关键帧候选最大距离
    float multi_matching_min_score{};  // 多分辨率地图匹配分值限制
    float ndt_matching_min_proba{};    // NDT验证时分值验证
    float ground_height{};             // 地面高度
    int num_threads = -1;              // 回环线程数量：-1时为系统最大线程数
};
} }  // namespace mapping::core

#endif  // MAPPING_MT_SEARCH_PARAM_H
