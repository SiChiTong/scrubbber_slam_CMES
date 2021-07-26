//
// Created by gaoxiang on 2020/9/17.
//

#ifndef MAPPING_GPS_STATUS_DEF_H
#define MAPPING_GPS_STATUS_DEF_H

#include "common/num_type.h"

namespace mapping::common {

/// GPS 状态定义
/// GPS状态在preprocessing阶段分析完成后，写入建图任务对应的yaml中，可以在后续阶段使用

/// 整体的GPS状态
/// 整体GPS状态取决于整条轨迹中的情况，如果轨迹当中存在稳定的GPS信号，那么认为整体GPS状态良好
enum class BagGNSSStatusType {
    BAG_GNSS_POS_HEAD_VALID_SYNC = 3,    // GPS位置与航向同时有效
    BAG_GNSS_POS_HEAD_VALID_UNSYNC = 2,  // GPS位置与航向有效，但不同时
    BAG_GNSS_EXIST = 0,                  // GPS有信号（单点或浮点解）
    BAG_GNSS_NOT_EXIST = -1,             // GPS无信号
};

/// 单个GPS读数的状态
enum class GpsStatusType {
    GNSS_FLOAT_SOLUTION = 5,         // 浮点解（cm到dm之间）
    GNSS_FIXED_SOLUTION = 4,         // 固定解（cm级）
    GNSS_PSEUDO_SOLUTION = 2,        // 伪距差分解（分米级）
    GNSS_SINGLE_POINT_SOLUTION = 1,  // 单点解（10m级）
    GNSS_NOT_EXIST = 0,              // GPS无信号
    GNSS_OTHER = -1,                 // 其他
};

inline bool GpsUsable(GpsStatusType gps_status) {
    return gps_status == GpsStatusType::GNSS_FIXED_SOLUTION || gps_status == GpsStatusType::GNSS_FLOAT_SOLUTION ||
           gps_status == GpsStatusType::GNSS_PSEUDO_SOLUTION || gps_status == GpsStatusType::GNSS_SINGLE_POINT_SOLUTION;
}

inline bool GpsLowAccuracy(BagGNSSStatusType gps_status) {
    return gps_status == BagGNSSStatusType::BAG_GNSS_POS_HEAD_VALID_UNSYNC ||
           gps_status == BagGNSSStatusType::BAG_GNSS_EXIST;
}

/// GPS状态和pose
struct GpsPoseStatus {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    GpsPoseStatus() = default;
    GpsPoseStatus(GpsStatusType status, const SE3& pose, bool heading_valid)
        : status_(status), pose_(pose), heading_valid_(heading_valid) {}

    GpsStatusType status_ = GpsStatusType::GNSS_NOT_EXIST;
    bool heading_valid_ = false;
    SE3 pose_;
};

/// GNSS 误差定义
constexpr double gnss_float_noise = 0.5;          // 浮点解精度
constexpr double gnss_fixed_noise = 0.15;         // 固定解精度
constexpr double gnss_pseudo_noise = 0.40;        // 伪距差分精度
constexpr double gnss_single_point_noise = 10.0;  // 单点解精度
constexpr double gnss_not_exist_noise = 1e6;      // 不存在时精度
constexpr double gnss_other_noise = 1e6;          // 其他

}  // namespace mapping::common

#endif  // MAPPING_GPS_STATUS_DEF_H
