//
// Created by wangqi on 19-7-19.
//

#ifndef MAPPING_MATCHING_PARAM_H
#define MAPPING_MATCHING_PARAM_H

//#include "io/yaml_io.h"
#include "common/yaml_io.h"

using namespace scrubber_slam;

namespace mapping { namespace core {

struct FeatureMatchingParams {
    void LoadFromYAML(const YAML_IO &yaml);

    int ground_scan_index = 7;
    float scan_period = 0.1;
    float ang_res_x = 0.2;
    float ang_res_y = 2.0;
    float ang_bottom = 15.1;
    float segment_alpha_x = 0.1;
    float segment_alpha_y = 2.0;
    float edge_th = 0.1;
    float surf_th = 0.1;
    float cloud_z_offset = 0.0; //1.17
    bool segment_enabled = false;
};

} }  // namespace mapping::core

#endif  // MAPPING_MATCHING_PARAM_H
