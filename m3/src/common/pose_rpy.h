//
// Created by gaoxiang on 2020/11/3.
//

#ifndef MAPPING_POSE_RPY_H
#define MAPPING_POSE_RPY_H

namespace mapping::common {
/// 以欧拉角表达的Pose
template <typename T>
struct Pose6D {
    Pose6D() = default;
    Pose6D(T xx, T yy, T zz, T r, T p, T y) : x(xx), y(yy), z(zz), roll(r), pitch(p), yaw(y) {}
    T x = 0;
    T y = 0;
    T z = 0;
    T roll = 0;
    T pitch = 0;
    T yaw = 0;
};

using PoseRPY = Pose6D<double>;
using PoseRPYF = Pose6D<float>;

}  // namespace mapping::common

#endif  // MAPPING_POSE_RPY_H
