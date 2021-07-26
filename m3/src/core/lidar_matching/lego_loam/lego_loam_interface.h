//
// Created by herenjie on 2021/5/10.
//

#ifndef MAPPING_LEGO_LOAM_INTERFACE_H
#define MAPPING_LEGO_LOAM_INTERFACE_H
#include "core/lidar_matching/lidar_matching.h"
#include "feature_association.h"
#include "core/lidar_matching/lego_loam/lego_loam_param.h"
#include "local_map.h"

namespace mapping::core {
class LegoLoamInterface : public LidarMatching {
   public:
//    LegoLoamInterface(LegoLoamParam& params);
    LegoLoamInterface();

    virtual ~LegoLoamInterface(){};

    virtual void SetStartID(int id) override;

    virtual void SetFirstFixedPose(const SE3& first_pose) override;

    virtual void SetInputCloud(const common::PointCloudType::Ptr& input, const int kf_id);

    virtual void Run() override;

    virtual void SetPredictPose(const SE3& predict_pose) override;

    virtual std::map<int, SE3> GetMatchingPoses() override;

    virtual std::map<int, V6d> GetMatchingNoise() override;

    virtual std::map<int, float> GetDegeneracyEigenValue() override;

   private:
//    LegoLoamParam params_;

    int trajectory_id_ = -1;
    int keyframe_id_ = -1;
    int start_id_ = -1;
    SE3 first_pose_;
    SE3 T_cur_last_; //  SE3 predict_pose = pre_kf->dr_pose_.inverse() * cur_kf->dr_pose_;

    struct lgFrame {
        lgFrame(CloudPtr input_cloud, int f_id)
            : origin_cloud(std::move(input_cloud)),
              id(f_id),
              corner_points_sharp_(new pcl::PointCloud<PointXYZI>()),
              corner_points_less_sharp_(new pcl::PointCloud<PointXYZI>()),
              surf_points_flat_(new pcl::PointCloud<PointXYZI>()),
              surf_points_less_flat_(new pcl::PointCloud<PointXYZI>()) {}

        CloudPtr origin_cloud = nullptr;
        CloudPtr corner_points_sharp_ = nullptr;
        CloudPtr corner_points_less_sharp_ = nullptr;
        CloudPtr surf_points_flat_ = nullptr;
        CloudPtr surf_points_less_flat_ = nullptr;
        int id;
        SE3 pose;  ///世界系位姿
    };
    std::shared_ptr<lgFrame> cur_frame_, last_frame_;

    std::deque<CloudPtr> local_map_corner_cloud_deque_;
    std::deque<CloudPtr> local_map_surf_cloud_deque_;
//    ros::Publisher pub_laser_cloud_corner_from_map_DS_;
//    ros::Publisher pub_laser_cloud_surf_from_map_DS_ros_;
    std::map<int, SE3> matching_poses_;
    std::map<int, V6d> matching_noises_;
    std::vector<bool> matching_degenerate_;
    std::map<int, float> degeneracy_eigenvalue_;

    std::shared_ptr<FeatureAssociation> fa_ptr_ = nullptr;
    std::shared_ptr<LocalMap> local_map_ = nullptr;
   private:
    void AddFeatureDeque(const std::shared_ptr<lgFrame>& cf);
};
}  // namespace mapping::core

#endif  // MAPPING_LEGO_LOAM_INTERFACE_H
