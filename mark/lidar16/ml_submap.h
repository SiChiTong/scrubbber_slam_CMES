//
// Created by gaoxiang on 2020/10/28.
//

#ifndef SCRUBBER_SLAM_ML_SUBMAP_H
#define SCRUBBER_SLAM_ML_SUBMAP_H

#include "common/num_types.h"
#include "common/point_type.h"
#include "lidar16/ml_frame.h"
#include "occupancy_mapping.h"

#include <pcl/common/transforms.h>
#include <opencv2/core.hpp>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

namespace scrubber_slam { namespace lidar16 {

/// 多线雷达使用的Submap
struct MLSubmap {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    /// 指定分辨率、宽度和高度，宽度和高度取物理单位
    MLSubmap(float resolution, float width, float height);
    MLSubmap() = default;
//    ~MLSubmap() = default;
    ~MLSubmap(){
//        cloud_->~PointCloud();
        keyframes_cur_submap_.clear();
        LOG(INFO)<<"SUBMAP DECONSTRUCT: "<<id_;
    };
    MLSubmap& operator=(const MLSubmap& submap);

/////////成员函数////////////////////////////
    bool Init();
    void ResetLocalPose(SE3 new_local_pose);

    /**
     * 新增一个关键帧
     * @param frame
     * @return
     */
    bool InsertKeyFrame(const std::shared_ptr<MLFrame>& frame);

    ///发布当前submap 的点云，坐标等
    void PubInfo();

    /// 带角度的坐标转换,///将当前栅格地图的像素坐标转换到世界坐标系
    Vec2f Image2WorldWithAngle(const Vec2i& pt_image);

    float Resolution() const { return resolution_; }

    void SetCenter(const Vec2f& c) {
        center_ = c;
        occupancy_mapping_->SetCenter(c);
    }
    void SetLocalPose(const SE3& local_pose) { local_pose_ = local_pose; }
    void SetLocalPoseOrigin(const SE3& local_pose) {
        local_pose_origin_ = local_pose;
    }

    Vec2f Center() const { return center_; }
    cv::Mat GetOccupancy() { return occupancy_mapping_->GetOccupancyImg(); }


    Vec2i World2Image(const Vec2f pt);
    bool OutSide(std::shared_ptr<MLFrame> frame);

    float MetricWidth() const { return metric_width_; }
    float MetricHeight() const { return metric_height_; }
    int ImageWidth() const { return image_width_; }
    int ImageHeight() const { return image_height_; }
    void SetTimeStamp(double timestamp) { timestamp_ = timestamp; }

    inline bool Contains(const Vec2& pt) {
        return pt[0] >= (center_[0] - metric_width_ * 0.5) && pt[0] < (center_[0] + metric_width_ * 0.5) &&
               pt[1] >= (center_[1] - metric_height_ * 0.5) && pt[1] < (center_[1] + metric_height_ * 0.5);
    }

    /**
     * 获取黑白图，允许缩放或转换为彩图
     * @param size
     * @return
     */
    cv::Mat GetOccupancyBW(int size = 500, bool convert_color = false);
    SE3 GetLocalPose(){
        std::unique_lock<std::mutex> spm(submap_pose_mutex_);
        return local_pose_;///当前submap的相对于世界坐标系的位姿，当前submap的第一帧位姿，会在pose graph之后更新位姿
    };
    SE3 GetLocalPoseOrigin(){
//        std::unique_lock<std::mutex> spm(submap_pose_mutex_);
        return local_pose_origin_;///当前submap的相对于世界坐标系的位姿，当前submap的第一帧位姿,不会变的那个
    };

//////不要单独再生成一个点云，大地图场景内存开销会很大
    CloudPtr GetCloud(){
            std::unique_lock<std::mutex> scm(submap_cloud_mutex_);
            return cloud_;
    }
    std::vector<std::shared_ptr<MLFrame>> GetKeyframes(){
        std::unique_lock<std::mutex> kcsm(keyframes_cur_submap_mutex_);
        return keyframes_cur_submap_;
    }

/////////成员变量////////////////////////////
    bool quit_;///析构，停止线程
    Idtype id_ = 0;///当前submap 的ID
///当前submap局部点云相关
    ///当前submap中的管理的关键帧和点云数据相关
    std::mutex submap_cloud_mutex_;
    //////不要单独再生成一个点云，大地图场景内存开销会很大
    CloudPtr cloud_ = nullptr;  /// 本submap对应的cloud, 放在当前submap局部坐标系下
 ///当前submap的局部点云地图保存位置
    std::string submap_pcd_path_;
    std::mutex keyframes_cur_submap_mutex_;
    std::vector<std::shared_ptr<MLFrame>> keyframes_cur_submap_;///当前submap中的所有的关键帧,一个submap可以有多个keyframes,一个keyframe只能出现在一个submap中

    ///当前submap位置相关，包括局部坐标，以及栅格地图位姿相关
    std::mutex submap_pose_mutex_;
    /*
     * 当前submap的相对于世界坐标系的位姿，当前submap的第一帧位姿,因为检测到回环并优化位姿之后，会更新submap的局部位姿，
     * 而后续的关键帧位姿是相对于最开始的那个位姿，这是由于是lego-loam接口求解位姿造成的，
     * 所以要记录下当前submap 的最开始的那一个位姿，用于求解后续关键帧和submap之间的相对位姿
     * */
    SE3 local_pose_;///当前submap的相对于世界坐标系的位姿，当前submap的第一帧位姿，会更新
    SE3 local_pose_origin_;///当前submap的相对于世界坐标系的位姿，当前submap的第一帧位姿，不会更新,仅仅通过相对位姿把点云转换到submap局部坐标系下

    double angle_ = 0.0;           // submap变化角度
    double first_kf_theta_ = 0.0;  // 第一个关键帧theta

    ///可视化相关,发布坐标系与当前submap坐标系下的点云
    bool init_publisher_ = false;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;///发布坐标系
//    ros::Publisher submap_points_publisher_;///发布当前点云用


    ///栅格地图相关
    std::shared_ptr<OccupancyMapping> occupancy_mapping_;///占据栅格地图应该由SUBMAP来管理
//    cv::Mat height_map_;        // 每个格子的占据高度
    float resolution_ = 0.05;
    double timestamp_ = 0;
    Vec2f center_ = Vec2f::Zero();        /// 图像物理中心,初始化本submap的第一帧点云的位置,在优化后更新submap位姿时也会更新它
    Vec2f center_image_ = Vec2f::Zero();  // 像素中心

    Vec2f kf_center_ = Vec2f::Zero();
    float kf_boundary_radious_ = 1.0;
    float theta_ = 0;  // 图像转角

    // 物理尺寸
    float metric_width_ = 0;
    float metric_height_ = 0;

    // 图像尺寸
    int image_width_ = 0;
    int image_height_ = 0;

    bool has_outside_points_ = false;

    bool fix_height_ = false;
};

} }  // namespace scrubber_slam::lidar16

#endif  // SCRUBBER_SLAM_ML_SUBMAP_H
