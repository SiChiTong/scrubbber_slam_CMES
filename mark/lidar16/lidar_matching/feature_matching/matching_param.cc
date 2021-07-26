//
// Created by wangqi on 19-7-21.
//
#include "lidar16/lidar_matching/feature_matching/matching_param.h"
#include "common/constants.h"

namespace mapping {
namespace core {

using namespace scrubber_slam;
using namespace mapping::common;

void FeatureMatchingParams::LoadFromYAML(const YAML_IO &yaml_file) {
    segment_enabled = yaml_file.GetValue<bool>("feature_matching_params", "segment_enabled");

    segment_alpha_x = ang_res_x / 180.0 * kPI;
    segment_alpha_y = ang_res_y / 180.0 * kPI;
    cloud_z_offset = 1.17;
}

}  // namespace core
}  // namespace mapping