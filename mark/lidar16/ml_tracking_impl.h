//
// Created by gaoxiang on 2020/10/29.
//

#ifndef SCRUBBER_SLAM_MLTRACKINGIMPL_H
#define SCRUBBER_SLAM_MLTRACKINGIMPL_H

#include <deque>
#include "common/lidar_tracking_status.h"
#include "lidar16/lego_loam_interface.h"
#include "lidar16/ml_frame.h"
#include "lidar16/ml_submap.h"
#include "lidar16/occupancy_mapping.h"
#include "tools/perception_interface/interface.h"
#include "lidar16/map_saver/map_saver.h"
#include "loop_closing/ml_loop_closing.h"
///dr 相关
#include "dr/dr_pre_integration.h"


#include <deque>
namespace scrubber_slam { namespace lidar16 {
enum trackingTypes{cartographer, lego_loam};
struct MLTrackingImpl {
    Idtype g_keyframe_id_;
    Idtype g_frame_id_;

    LidarTrackingStatus status_ = LidarTrackingStatus::INIT;
    double current_lidar_time_ = 0;
    double last_lidar_time_ = 0;

    std::shared_ptr<MLFrame> current_frame_ = nullptr;
    std::shared_ptr<MLFrame> last_frame_ = nullptr;
    std::shared_ptr<MLFrame> last_keyframe_ = nullptr;
    std::mutex all_kf_mutex_;
    std::vector<std::shared_ptr<MLFrame>> all_keyframes_;
    std::deque<std::shared_ptr<MLFrame>> history_keyframes_;  // keyframes 队列

    trackingTypes tracking_types_ = cartographer;
/// lego-loam 操作界面相关，用于帧间匹配，前端tracking, TODO: 把lego-loam需要的特征点点云放到对应的关键帧里管理
    std::shared_ptr<LegoLoamInterface> lego_interface_;

    std::deque<CloudPtr> local_map_corner_cloud_deque_;  ///关键帧队列，用于局部地图匹配
    std::deque<CloudPtr> local_map_surf_cloud_deque_;    ///关键帧队列，用于局部地图匹配
    std::deque<CloudPtr> local_map_ground_cloud_deque_;  ///关键帧队列，用于局部地图匹配

///IMU轮速相关
    std::shared_ptr<DRPreIntegration> dr_pre_integration_;
    std::mutex id_deque_mutex_;
    std::deque<UImuMsgPtr> imu_deque_;
    std::mutex ws_deque_mutex_;
    std::deque<FrontWheelSpeedMsgScru::ConstPtr> wheelspeed_deque_;

    std::shared_ptr<mapping::tools::PerceptionInterface> perception_interface_;  //
    std::shared_ptr<MapSaver> map_saver_;  //点云切片地图保存，后续会用于定位

///地图相关
    SE3 origin_pose_;                   // 第一帧点云的位置,用于在拓展地图时，确定第一帧激光在老地图中的位姿
    Vec2 origin_point_ = Vec2::Zero();  // 地图原点,全局地图物理原点，第一帧点云的坐标位置
    std::shared_ptr<MLSubmap> current_submap_ = nullptr;///占据栅格地图用的submap
    std::shared_ptr<MLSubmap> current_closeloop_submap_ = nullptr;///回环检测用的submap，数量超过150或者在不同的占据栅格的submap,就会新建一个submap
    std::vector<std::shared_ptr<MLSubmap>> all_closeloop_submaps_;///所有的submap按时间（ID）顺序依次排列的的submap

    std::mutex id_submap_mutex_;
    std::map<Idtype, std::shared_ptr<MLSubmap>> submaps_by_id_;//回环检测用的submap的ID和submap
    std::mutex kf_id_mutex_;
    std::map<IdType, std::shared_ptr<MLFrame>> keyframes_by_id_;             // keyframes by keyframe id

    Idtype submap_id_ = 0; ///submap 的ID
    const float submap_size_ = 100;           // 每张submap大小
//    float model_size_ = 20.0;                 // 模板物理大小
    float resolution_ = 0.05;                 // 分辨率


    ///回环检测
    std::shared_ptr<MLLoopClosing> loop_closure_;
    std::thread loop_closure_thread_;
    std::thread submap_thread_;
    bool quit_;

    SE3 T_IMU_lidar_;  // IMU到lidar外参
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_br_;///发布坐标系

///功能点<功能点名称，　功能点对应的关键帧ＩＤ>
    std::deque<std::string> func_to_set_;
    std::unordered_map<std::string, int>  all_func_pts_;

///可视化调试用
    bool use_self_visualization_ = false;
    bool using_front_wheelspeed_ = false;
    std::string submap_pcd_path_;

    bool if_show_localization_map_ = false;///将当前地图发布到ROS topic 上，可视化调试用
    CloudPtr result_cloud_ptr_;///当前构建的地图，用于可视化调试
    ros::Publisher pub_current_map_points_;

    ros::Publisher pub_sharp_points_;
    ros::Publisher pub_less_sharp_points_;
    ros::Publisher pub_flat_points_;
    ros::Publisher pub_less_flat_points_;


    ros::Publisher pub_pose_;
    ros::Publisher pub_pose_DR_;
    std::thread pub_mapping_thread_;

    std::mutex start_position_mutex_;
    Vec2 start_position_;
    bool back_to_start_position_ = false;
    double move_dist_ = 0.0;
};
} }  // namespace scrubber_slam::lidar16

#endif  // SCRUBBER_SLAM_MLTRACKINGIMPL_H
