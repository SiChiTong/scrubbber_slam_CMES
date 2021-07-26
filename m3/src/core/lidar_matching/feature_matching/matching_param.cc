//
// Created by wangqi on 19-7-21.
//
#include "core/lidar_matching/feature_matching/matching_param.h"
#include "common/constants.h"

namespace mapping {
namespace core {

using namespace mapping::common;

void FeatureMatchingParams::LoadFromYAML(const io::YAML_IO &yaml_file) {
    set_height_zero = yaml_file.GetValue<bool>("feature_matching_params", "set_height_zero");
    loop_closure_enable =
        yaml_file.GetValue<bool>("feature_matching_params", "loop_closure_enable");
    segment_enabled = yaml_file.GetValue<bool>("feature_matching_params", "segment_enabled");
    loop_process_time_interval =
        yaml_file.GetValue<double>("feature_matching_params", "loop_process_time_interval");
    loop_near_search_dist_scope =
        yaml_file.GetValue<double>("feature_matching_params", "loop_near_search_dist_scope");
    loop_near_search_time_interval =
        yaml_file.GetValue<double>("feature_matching_params", "loop_near_search_time_interval");
    loop_fitness_score_th =
        yaml_file.GetValue<double>("feature_matching_params", "loop_fitness_score_th");

    segment_alpha_x = ang_res_x / 180.0 * kPI;
    segment_alpha_y = ang_res_y / 180.0 * kPI;
    cloud_z_offset = 1.17;
}

}  // namespace core
}  // namespace mapping