//
// Created by gaoxiang on 2020/10/28.
//

#ifndef SCRUBBER_SLAM_LEGO_LOAM_INTERFACE_H
#define SCRUBBER_SLAM_LEGO_LOAM_INTERFACE_H

#include "common/global_config.h"
#include "lidar16/ml_frame.h"

#include "lidar_matching/lego_loam/feature_association.h"
#include "lidar_matching/lego_loam/image_projection.h"
//#include "lidar_matching/lego_loam/local_map.h"
#include "lidar_matching/feature_matching/local_mapper.h"
namespace scrubber_slam { namespace lidar16 {

/// Loam 界面
class LegoLoamInterface {
   public:
    LegoLoamInterface();
    ~LegoLoamInterface() = default;

    bool clear(){ };

    /// 使用Lego-loam 计算一个帧的pose
    ///拆分lego-loam
    void FrameFeatureExtract(const std::shared_ptr<MLFrame>& frame);
    void FrameFeatureTracker(const std::shared_ptr<MLFrame>& pre_frame, const std::shared_ptr<MLFrame>& cur_frame);
    void FrameLocalMapper( const std::deque<CloudPtr>& local_map_corner_cloud_deque,
                           const std::deque<CloudPtr>& local_map_surf_cloud_deque,
                           const std::deque<CloudPtr>& local_map_ground_cloud_deque,
                           const std::shared_ptr<MLFrame>& cur_frame );

   private:
    ///拆分lego-loam
    std::shared_ptr<FeatureAssociation> fa_ptr_;
//    std::shared_ptr<LocalMap> local_map_;
  std::shared_ptr<mapping::core::LocalMapper> local_mapper_ = nullptr;

  std::shared_ptr<MLFrame> last_frame_ ;

    std::deque<std::shared_ptr<MLFrame>> kf_deque_;///局部地图关键帧
    ros::Publisher pub_laser_cloud_corner_from_map_DS_;
    ros::Publisher pub_laser_cloud_surf_from_map_DS_ros_;

    bool fix_height_ = false;
    int keyframe_num_ = 0;
};

} }  // namespace scrubber_slam::lidar16

#endif  // SCRUBBER_SLAM_LEGO_LOAM_INTERFACE_H
