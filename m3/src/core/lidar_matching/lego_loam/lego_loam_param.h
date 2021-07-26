//
// Created by wangqi on 19-7-19.
//

#ifndef LEGO_LOAM_PARAM_H
#define LEGO_LOAM_PARAM_H

#include "io/yaml_io.h"

namespace mapping::core {

struct LegoLoamParam {
    void LoadFromYAML(const io::YAML_IO &yaml);

    int N_SCAN = 16;
    const int Horizon_SCAN = 1800;
    const float ang_res_x = 0.2;
    const float ang_res_y = 2.0;
    const float ang_bottom = 15.0+0.1;
    const int groundScanInd = 7;

};

}  // namespace mapping::core

#endif  // MAPPING_MATCHING_PARAM_H
