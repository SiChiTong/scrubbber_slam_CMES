//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_TRACK_POSE_H
#define MAPPING_TRACK_POSE_H

#include "common/num_type.h"

namespace mapping::common {

/// 贴边轨迹
template <typename T>
struct TrackPose {
    TrackPose(int i, const Eigen::Matrix<T, 3, 1>& xyz, float ndt_score)
        : id(i), x(xyz[0]), y(xyz[1]), z(xyz[2]), ndt_filter(ndt_score) {}
    TrackPose() = default;
    T x = 0;
    T y = 0;
    T z = 0;
    float ndt_filter = 0;
    int id = 0;
};

using TrackPoseD = TrackPose<double>;
using TrackPoseF = TrackPose<float>;

}  // namespace mapping::common

#endif  // MAPPING_TRACK_POSE_H
