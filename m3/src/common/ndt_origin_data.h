//
// Created by gaoxiang on 2021/2/1.
//

#ifndef MAPPING_FILTER_NDT_SCORE_H
#define MAPPING_FILTER_NDT_SCORE_H

namespace mapping {
namespace common {

struct NdtOriginData {
    double header_time = 0;
    float xg = 0;
    float yg = 0;
    float distance = 0;
    float trans_probability = 0;
    long index = 0;
    int threshold_index = 0;
    NdtOriginData(float x, float y, float score) : xg(x), yg(y), trans_probability(score) {}
    NdtOriginData() = default;
};
}  // namespace common
}  // namespace mapping

#endif  // MAPPING_FILTER_NDT_SCORE_H
