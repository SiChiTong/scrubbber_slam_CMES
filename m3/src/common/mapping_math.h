//
// Created by gaoxiang on 2020/8/10.
//

#ifndef MAPPING_MAPPING_MATH_H
#define MAPPING_MAPPING_MATH_H

#include <cmath>

#include "common/constants.h"
#include "common/num_type.h"
#include "common/pose_rpy.h"

#include "tf/LinearMath/Matrix3x3.h"

/// 常用的数值运算
namespace mapping::common {

inline double SINL(double lat) { return std::sin(lat * kDEG2RAD); }
inline double COSL(double lat) { return std::cos(lat * kDEG2RAD); }
inline double TANL(double lat) { return std::tan(lat * kDEG2RAD); }
inline double SIN2L(double lat) { return std::sin(2.0 * lat * kDEG2RAD); }

inline double SQ(double x) { return (x * x); }
inline double RM(double lat) { return (kWGS84_RE * (1 - 2.0 * kWGS84_F + 3 * kWGS84_F * SQ(SINL(lat)))); }
inline double RN(double lat) { return (kWGS84_RE * (1 + kWGS84_F * SQ(SINL(lat)))); }

inline Quat Att2Quat(const V3d &att) {
    double cosR = cos(att(0) / 2.0);
    double sinR = sin(att(0) / 2.0);
    double cosP = cos(att(1) / 2.0);
    double sinP = sin(att(1) / 2.0);
    double cosH = cos(att(2) / 2.0);
    double sinH = sin(att(2) / 2.0);

    double qw = cosH * cosP * cosR + sinH * sinP * sinR;
    double qx = cosH * sinP * cosR + sinH * cosP * sinR;
    double qy = cosH * cosP * sinR - sinH * sinP * cosR;
    double qz = cosH * sinP * sinR - sinH * cosP * cosR;
    return Quat(qw, qx, qy, qz);
}

inline void Att2Cbn(M3d &cbn, const V3d &att) {
    double cosR = cos(att(0));
    double sinR = sin(att(0));
    double cosP = cos(att(1));
    double sinP = sin(att(1));
    double cosH = cos(att(2));
    double sinH = sin(att(2));

    cbn(0, 0) = cosR * cosH + sinR * sinP * sinH;
    cbn(0, 1) = cosP * sinH;
    cbn(0, 2) = sinR * cosH - cosR * sinP * sinH;
    cbn(1, 0) = -cosR * sinH + sinR * sinP * cosH;
    cbn(1, 1) = cosP * cosH;
    cbn(1, 2) = -sinR * sinH - cosR * sinP * cosH;
    cbn(2, 0) = -sinR * cosP;
    cbn(2, 1) = sinP;
    cbn(2, 2) = cosR * cosP;
}

inline V3d Quat2Att(const Quat &q) {
    double q00 = q.w() * q.w();
    double q01 = q.w() * q.x();
    double q02 = q.w() * q.y();
    double q03 = q.w() * q.z();
    double q11 = q.x() * q.x();
    double q12 = q.x() * q.y();
    double q13 = q.x() * q.z();
    double q22 = q.y() * q.y();
    double q23 = q.y() * q.z();
    double q33 = q.z() * q.z();

    V3d att;
    att(0) = atan2(-2.0 * (q13 - q02), q00 - q11 - q22 + q33);
    att(1) = asin(2.0 * (q01 + q23));
    att(2) = atan2(2.0 * (q12 - q03), q00 - q11 + q22 - q33);
    return att;
}

/// 从pose中取出yaw pitch roll
/// 使用的顺序是：roll pitch yaw ，最左是roll
/// 由于欧拉角的多解性，Eigen在分解时选择最小化第一个（也就是roll）。否则，如果选择最小化yaw，那么roll和pitch可能出现大范围旋转，不符合实际
/// @see https://stackoverflow.com/questions/33895970/about-eulerangles-conversion-from-eigen-c-library
/// 这个用在lego-loam的lidar odom里，符合它的定义
/// 2020.11 change back to tf to keep consist
inline common::PoseRPY SE3ToRollPitchYaw(const SE3 &pose) {
    auto rot = pose.rotationMatrix();
    tf::Matrix3x3 temp_tf_matrix(rot(0, 0), rot(0, 1), rot(0, 2), rot(1, 0), rot(1, 1), rot(1, 2), rot(2, 0), rot(2, 1),
                                 rot(2, 2));
    PoseRPY output;
    output.x = pose.translation()[0];
    output.y = pose.translation()[1];
    output.z = pose.translation()[2];

    temp_tf_matrix.getRPY(output.roll, output.pitch, output.yaw);
    return output;
}

/// 从xyz和yaw pitch roll转成SE3
/// 使用0,1,2顺序（从右读）,yaw最先旋转
inline SE3 XYZRPYToSE3(const common::PoseRPY &pose) {
    using AA = Eigen::AngleAxisd;
    return SE3((AA(pose.yaw, V3d::UnitZ()) * AA(pose.pitch, V3d::UnitY()) * AA(pose.roll, V3d::UnitX())),
               V3d(pose.x, pose.y, pose.z));
}

/**
 * pose 插值算法
 * @tparam T    数据类型
 * @param query_time 查找时间
 * @param data  数据
 * @param take_pose_func 从数据中取pose的谓词
 * @param result 查询结果
 * @param best_match_iter 查找到的最近匹配
 *
 * NOTE 要求query_time必须在data最大时间和最小时间之间
 * data的map按时间排序
 * @return
 */
template <typename T>
bool PoseInterp(double query_time, const std::map<double, T> &data, const std::function<SE3(const T &)> &take_pose_func,
                SE3 &result, T &best_match) {
    if (data.empty()) {
        return false;
    }

    if (query_time > data.rbegin()->first) {
        return false;
    }

    auto match_iter = data.begin();
    for (auto iter = data.begin(); iter != data.end(); ++iter) {
        auto next_iter = iter;
        next_iter++;

        if (iter->first < query_time && next_iter->first >= query_time) {
            match_iter = iter;
            break;
        }
    }

    auto match_iter_n = match_iter;
    match_iter_n++;
    assert(match_iter_n != data.end());

    double dt = match_iter_n->first - match_iter->first;
    double s = (query_time - match_iter->first) / dt;  // s=0 时为第一帧，s=1时为next

    SE3 pose_first = take_pose_func(match_iter->second);
    SE3 pose_next = take_pose_func(match_iter_n->second);
    result = {pose_first.unit_quaternion().slerp(s, pose_next.unit_quaternion()),
              pose_first.translation() * (1 - s) + pose_next.translation() * s};
    best_match = s < 0.5 ? match_iter->second : match_iter_n->second;
    return true;
}

}  // namespace mapping::common

#endif  // MAPPING_MAPPING_MATH_H
