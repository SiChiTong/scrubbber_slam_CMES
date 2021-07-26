//
// Created by wangqi on 19-7-17.
//

#ifndef MAPPING_KEYFRAME_H
#define MAPPING_KEYFRAME_H

#include "common/gps_status_def.h"
#include "common/mapping_point_types.h"
#include "common/num_type.h"

#include <map>
#include <memory>

namespace mapping::common {

/// 每个关键帧所属的Bag类型
enum class KeyFrameBagType {
    MAPPING_BAGS = 0,  // d-bags 这些包用于建图，建完之后存在map.db里
    VALIDATION_BAGS,  // z-bags or 其他包，轨迹会存储于map.db中，但点云不被存储。为了验证，会把点云放在val.db中
};

/// 单帧关键帧
/// keyframe是激光前端与后端处理的基础数据单元
/// 使用keyframe的指针进行操作，以让点云不被重复使用
struct KeyFrame {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    KeyFrame() { SetDefaultNoise(); }

    KeyFrame(const KeyFrame& kf) = delete;
    bool operator=(const KeyFrame& kf) = delete;

    void SetDefaultNoise() {
        matching_noise_ << 0.05, 0.05, 0.05, 0.008, 0.008, 0.008;  // 5cm and 0.5 degrees
        dr_noise_ << 0.05, 0.05, 0.05, 0.008, 0.008, 0.008;        // 5cm and 0.5 degrees
        gps_noise_ << 0.15, 0.15, 0.15, 0.08, 0.08, 0.08;          // 15 cm and 5 degrees
    }

    /// 根据GPS状态设置噪声矩阵
    void SetGpsNoiseFromStatus();

    /// 卸载点云，清空内存
    inline void UnloadCloud() { cloud_ = nullptr; }

    IdType id_ = 0;                                             // keyframe自己的Id
    IdType trajectory_id_ = 0;                                  // 轨迹id
    double timestamp_ = 0;                                      // 时间戳
    PointCloudType::Ptr cloud_ = nullptr;                       // 点云
    KeyFrameBagType bag_type_ = KeyFrameBagType::MAPPING_BAGS;  // 所属的包类型

    /// 各种pose和noise
    /// 激光
    SE3 matching_pose_;
    V6d matching_noise_ = V6d::Zero();

    /// DR
    SE3 dr_pose_;
    V6d dr_noise_ = V6d::Zero();

    /// GPS
    GpsStatusType gps_status_ = GpsStatusType::GNSS_NOT_EXIST;  // GPS状态
    SE3 gps_pose_;                                              // GPS测量
    V6d gps_noise_ = V6d::Zero();                               // GPS噪声
    bool heading_valid_ = false;                                // GPS heading 是否有效
    bool gps_inlier_ = true;                                    // GPS 是否为正常值（由第一轮优化判定）

    /// 后端用的变量
    float matching_eigen_value_ = 200;  // 退化分值，取100为阈值，默认值为200
    /// 最终优化pose
    SE3 optimized_pose_stage_1_;  // 第一阶段优化
    SE3 optimized_pose_stage_2_;  // 第二阶段优化
};

using KFPtr = std::shared_ptr<KeyFrame>;                          // shared_ptr to keyframe
using KFMapType = std::map<IdType, KFPtr>;                        // keyframe map by id
using KFTrajType = std::map<IdType, std::vector<common::KFPtr>>;  // 以轨迹ID排序的keyframes

}  // namespace mapping::common

#endif  // MAPPING_KEYFRAME_H
