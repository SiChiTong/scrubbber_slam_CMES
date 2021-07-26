//
// Created by wangqi on 19-7-19.
//

#ifndef MAPPING_MATCHING_PARAM_H
#define MAPPING_MATCHING_PARAM_H

#include "io/yaml_io.h"

namespace mapping::core {

struct FeatureMatchingParams {
    void LoadFromYAML(const io::YAML_IO &yaml);

    int ground_scan_index = 7;
    float scan_period = 0.1;
    float ang_res_x = 0.2;
    float ang_res_y = 2.0;
    float ang_bottom = 15.1;
    float segment_alpha_x = 0.1;
    float segment_alpha_y = 2.0;
    float edge_th = 0.1;
    float surf_th = 0.1;
    float cloud_z_offset = 1.17;
    float nearest_feature_search_dist = 25;
    float surrounding_keyframe_search_radius = 50;
    int surrounding_keyframe_search_num = 20;
    float keyframe_delta_th = 0.3;
    bool set_height_zero;
    bool loop_closure_enable;
    bool segment_enabled;
    double loop_process_time_interval;
    double loop_near_search_dist_scope;
    double loop_near_search_time_interval;
    double loop_fitness_score_th;
};

}  // namespace mapping::core

#endif  // MAPPING_MATCHING_PARAM_H
