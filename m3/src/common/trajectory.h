//
// Created by gaoxiang on 2020/8/21.
//

#ifndef MAPPING_TRAJECTORY_H
#define MAPPING_TRAJECTORY_H

#include "common/function_point.h"
#include "common/keyframe.h"
#include "common/message_def.h"
#include "common/num_type.h"

#include <chrono>

namespace mapping ::common {

/// 以轨迹为单位的存储方式
/// 由于自动分包功能，一条轨迹可由多个bag包拼接而成,bag包命名规则与采集时相同
struct Trajectory {
    using TimeT = tm;
    explicit Trajectory();
    Trajectory(IdType id, const std::string& bag_name);

    IdType trajectory_id = 0;                                   // 轨迹id
    std::string bag_name;                                       // 不含日期的包名
    std::map<IdType, std::string> bag_files;                    // 按part先后顺序排序的bag files
    std::vector<std::shared_ptr<common::KeyFrame>> keyframes_;  // 本条轨迹的keyframes
    bool is_mapping_bag_;                                       // 是否为建图数据轨迹（d包）

    /// trajectory对应的消息，激光数据比较大，在计算轨迹之后才会读取，然后存储到DB中
    std::map<double, ImuMsgPtr> imu_msgs_;
    std::map<double, OdomMsgPtr> odom_msgs_;
    std::map<double, CanMsgPtr> can_msgs_;
    std::map<double, GpsMsgPtr> gps_msgs_;

    // 如果为z包，那么还有功能点
    std::vector<common::FunctionPoint> function_points_;  // 功能点
};

Trajectory::TimeT BagDate2Time(const std::string& bag_time);

}  // namespace mapping::common

#endif  // MAPPING_TRAJECTORY_H
