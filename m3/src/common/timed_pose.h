//
// Created by gaoxiang on 2020/9/2.
//

#ifndef MAPPING_TIMED_POSE_H
#define MAPPING_TIMED_POSE_H

#include "common/num_type.h"

namespace mapping::common {

/// 带时间的pose
struct TimedPose {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    TimedPose() = default;
    TimedPose(double timestamp, const SE3& p) : time(timestamp), pose(p){};

    double time = 0;
    SE3 pose;

    bool operator<(const TimedPose& timed_pose) const { return ((time - timed_pose.time) < 0); };
};

}  // namespace mapping::common

#endif  // MAPPING_TIMED_POSE_H
