//
// Created by gaoxiang on 2020/10/28.
//

#ifndef SCRUBBER_SLAM_ML_TRACKING_H
#define SCRUBBER_SLAM_ML_TRACKING_H

#include "common/num_types.h"
#include "common/point_type.h"
#include "common/message_def.h"
#include "ml_submap.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include "common/num_type.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>



namespace scrubber_slam {
class DeadReckoning;
}

namespace scrubber_slam { namespace lidar16 {
struct MLTrackingImpl;
class LegoLoamInterface;
class MLLoopClosing;

/// 多线激光前端：使用Lego-loam计算激光pose，管理关键帧，创建并管理submap
class MLTracking {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    MLTracking();
    ~MLTracking();

    bool Init();

    ///更新地图要用
    void SetFirstPose(const SE3& pose);
    void GetFirstPose(SE3& pose);
    bool GetGlobalOccuMapImage(cv::Mat& global_map_out, Vec2f& top_left, Vec2f& bottom_right);///用于拓展地图时获取全局栅格地图
    bool GetGlobalCloudMap(CloudPtr& cloud_old);///用于拓展地图时获取全局点云地图

    void TransPacakge2cloudXYZIHIRBS(const SE3& pose_from_dr_l, const PacketsMsg::ConstPtr& lidar_msg,double timestamp,
                                     mapping::common::PointCloudPtr cloud_XYZIHIRBS);
    bool PreprocessCloud(double timestamp, const PacketsMsg::ConstPtr& lidar_msg);

    bool SaveMap(const std::string& data_path);///保存点云地图和栅格地图，业务发过啦的保存地图服务调的是这个函数
    bool SaveOccupancyMap(const std::string &path);///保存栅格地图
    ///计算栅格地图，供显示和保存地图用
    bool ComputeOccupancyMap(cv::Mat& global_map_img_save,
                             cv::Mat& global_map_img_traj_show,
                             Vec2f& top_left, Vec2f& bottom_right);
    cv::Mat GetGlobalMapImage();///用于建图时显示用的图


    void Quit();

//   private:

    void AddKeyFrame();
    void AddKeyFrameToSubmap();
    void CreateSubmap(const SE3 &center);

    void PubMap();///用于发布当前点云地图，可视化调试

    void Reset() {}

    bool IsKeyFrame();
    void AddFeatureDeque(const std::shared_ptr<MLFrame>& cf);

    void UpdateSubMapPose(Idtype submap_id, SE3 local_pose);
    std::map<Idtype, std::shared_ptr<MLSubmap>> GetAllSubMap();//回环检测用的submap的ID和submap
    std::map<IdType, std::shared_ptr<MLFrame>> GetKeyframByID();

    bool GetBackToStartPoint();

    bool GetCurPose(SE3 & cur_pose);
    bool SetFuncPoint(const std::string& func_name);
    bool GetAllFuncPts(std::vector<geometry_msgs::PoseStamped> &func_pts_vec);

    void AddNewImu(const UImuMsgPtr& imu);
    void AddNewFrontWheelSpeed(const FrontWheelSpeedMsgScru::ConstPtr &wheelspeed);
    void AddNewRearWheelSpeed(UWheelSpeedMsgPtr wheelspeed);

    std::vector<std::shared_ptr<MLFrame>> GetHistoryKeyFrames();

    std::unique_ptr<MLTrackingImpl> impl_;
};
} }  // namespace scrubber_slam::lidar16

#endif  // SCRUBBER_SLAM_ML_TRACKING_H
