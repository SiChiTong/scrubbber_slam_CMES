//
// Created by gaoxiang on 2020/10/28.
//

#ifndef SCRUBBER_SLAM_ML_FRAME_H
#define SCRUBBER_SLAM_ML_FRAME_H

#include "common/message_def.h"
#include "common/num_types.h"
#include "common/point_type.h"
#include "common/mapping_point_types.h"
#include "common/timed_pose.h"

namespace scrubber_slam { namespace lidar16 {

/// 多线激光雷达组成的frame
struct MLFrame {
    MLFrame() = default;
    ~MLFrame(){
        cloud_ptr_ = nullptr;
    }
    static std::shared_ptr<MLFrame> CreateMLFrame(CloudPtr cloud);
    static std::shared_ptr<MLFrame> CreateMLFrame(const mapping::common::PointCloudPtr& cloudXYZIHIRBS);
    static void SetIdcountZero();
    void SetPose(SE3 pose_lidar){
        Twl_ = pose_lidar;
    };

    /// 2D位置（IMU)
    Vec2 xy() const { return Twi_.translation().head<2>(); }
    ///2D位置 （LIDAR）
    Vec2 xy_l() const { return Twl_.translation().head<2>(); }
    /// 2D yaw
    double theta() const { return Twi_.so3().matrix().eulerAngles(0,1,2)[0]; }
    double theta_l() const { return Twl_.so3().matrix().eulerAngles(0,1,2)[0]; }

    Idtype id_ = 0;
    Idtype keyframe_id_ = 0;
    bool is_keyframe_ = false;
    double stamp_;  // 时间戳

    double degeneracy_score_ = 0.0;  // 退化分值，越接近于0越退化

    // NOTE 定位输出的是IMU pose
    SE3 last_Twi_;  /// world to imu, from dr，这个是imu在世界坐标系的位姿，他们用的位姿，可在回环优化后等情况随时更新位姿,现在只是在获取全部关键帧用于得到建图轨迹是更新了
    SE3 Twi_;  /// world to imu, from dr，这个是imu在世界坐标系的位姿，他们用的位姿，可在回环优化后等情况随时更新位姿,现在只是在获取全部关键帧用于得到建图轨迹是更新了
    SE3 Twl_;  /// world to lidar, from lidar,这个是lego_loam求出来的位姿，相对的，这个在leog_loam求出来那一刻就固定了，不能改
    SE3 Twl_dr_;  // world to lidar, from dr
    /// 当前帧点云转换到submap局部坐标系下,当前帧的点云可以通过这个变换到submap 的局部坐标系下
    /// NOTE: 对于每一帧关键帧，Tsl_（点云和局部坐标系的关系）是固定不变的，
    /// 通过自己的Twl_和所在的submap的第一关键帧的Twl_的差求得值，且从此固定不变
    SE3 Tsl_;


    SE3 T_kf2w_;


    double lego_loam_score_ = 0.;

    CloudPtr cloud_ptr_ = nullptr;  // 点云
    mapping::common::PointCloudPtr cloudXYZIHIRBS_ptr_ = nullptr;  // 点云

    ///用于回环检测的参数
    Idtype kf_in_submap_id_;///本关键帧所在的submap的ID

    ///拆分lego-loam,lego-loam的特征点
//    mapping::common::PointCloudType::Ptr corner_points_sharp_;
//    mapping::common::PointCloudType::Ptr corner_points_less_sharp_;
//    mapping::common::PointCloudType::Ptr surf_points_flat_;
//    mapping::common::PointCloudType::Ptr surf_points_less_flat_;
//    mapping::common::PointCloudType::Ptr outlier_cloud_;
    CloudPtr corner_points_sharp_;
    CloudPtr corner_points_less_sharp_;
    CloudPtr surf_points_flat_;
    CloudPtr surf_points_less_flat_;
    CloudPtr ground_cloud_;

    ///当前关键帧作为功能点
    bool is_func_point_ = false;
    std::string func_name_;
};

} }  // namespace scrubber_slam::lidar16

#endif  // SCRUBBER_SLAM_FRAME_H
