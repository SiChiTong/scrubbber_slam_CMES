//
// Created by gaoxiang on 2020/10/28.
//

#include "lego_loam_interface.h"
#include <core/block_solver.h>
#include <core/optimization_algorithm_levenberg.h>
#include <core/robust_kernel_impl.h>
#include <core/sparse_optimizer.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <solvers/dense/linear_solver_dense.h>
#include <types/slam3d/edge_se3.h>
#include <types/slam3d/edge_se3_prior.h>
#include <types/slam3d/vertex_se3.h>

namespace scrubber_slam {
namespace lidar16 {
LegoLoamInterface::LegoLoamInterface() {
    ///拆分lego-loam
    ros::NodeHandle nh;
    pub_laser_cloud_corner_from_map_DS_ = nh.advertise<sensor_msgs::PointCloud2>("/tracking/cloud_corner_map", 1);
    pub_laser_cloud_surf_from_map_DS_ros_ = nh.advertise<sensor_msgs::PointCloud2>("/tracking/cloud_surfm_map", 1);

    fa_ptr_ = std::make_shared<FeatureAssociation>(nh);
    //    local_map_ = std::make_shared<LocalMap>();
    mapping::core::FeatureMatchingParams params;
    auto yaml = GlobalConfig::Get();
    fix_height_ = yaml->GetValue<bool>("system", "fix_height");

    params.LoadFromYAML(*yaml);
    local_mapper_ = std::make_shared<mapping::core::LocalMapper>(params);

    LOG(ERROR) << "LegoLoamInterface: ----------------------------------" << this;
}

///拆分lego-loam
// 1.提取特征点
void LegoLoamInterface::FrameFeatureExtract(const std::shared_ptr<MLFrame>& frame) {
    fa_ptr_->InsertLidar(frame->cloud_ptr_);
    CloudPtr sharp_cloud(new PointCloudType), less_sharp_cloud(new PointCloudType), flat_cloud(new PointCloudType),
        less_flat_cloud(new PointCloudType), ground_cloud(new PointCloudType);

    fa_ptr_->GetSegCloud(sharp_cloud, less_sharp_cloud, flat_cloud, less_flat_cloud, ground_cloud);

    *frame->corner_points_sharp_ = *sharp_cloud;
    *frame->corner_points_less_sharp_ = *less_sharp_cloud;
    *frame->surf_points_flat_ = *flat_cloud;
    *frame->surf_points_less_flat_ = *less_flat_cloud;
    *frame->ground_cloud_ = *ground_cloud;
}
// 2.帧间匹配
void LegoLoamInterface::FrameFeatureTracker(const std::shared_ptr<MLFrame>& pre_frame,
                                            const std::shared_ptr<MLFrame>& cur_frame) {}
// 3.local map
void LegoLoamInterface::FrameLocalMapper(const std::deque<CloudPtr>& local_map_corner_cloud_deque,
                                         const std::deque<CloudPtr>& local_map_surf_cloud_deque,
                                         const std::deque<CloudPtr>& local_map_ground_cloud_deque,
                                         const std::shared_ptr<MLFrame>& cur_frame) {
    SE3 opt_pose_kf_w = cur_frame->Twl_;

    ///当前帧的一些特征点： corner、surf、ground
    CloudPtr corner_cur_scan(new PointCloudType);
    CloudPtr surf_cur_scan(new PointCloudType);
    CloudPtr ground_cur_scan(new PointCloudType);
    *corner_cur_scan = *cur_frame->corner_points_sharp_ + *cur_frame->corner_points_less_sharp_;
    *surf_cur_scan = *cur_frame->surf_points_flat_ + *cur_frame->surf_points_less_flat_;
    *ground_cur_scan = *cur_frame->ground_cloud_;

    ///局部地图中的一些特征点： corner、surf、ground
    auto sum_local_cloud = [](const std::deque<CloudPtr>& local_cloud_deque) -> CloudPtr {
        CloudPtr cloud_sum(new PointCloudType);
        for (auto& cloud : local_cloud_deque) *cloud_sum += *cloud;
        return cloud_sum;
    };
    CloudPtr corner_local_map = sum_local_cloud(local_map_corner_cloud_deque);
    CloudPtr surf_local_map = sum_local_cloud(local_map_surf_cloud_deque);
    CloudPtr ground_local_map = sum_local_cloud(local_map_ground_cloud_deque);

    auto pub_cloud = [](const ros::Publisher& publisher, const CloudPtr& cloud) {
        sensor_msgs::PointCloud2 cloud_ros;
        pcl::toROSMsg(*cloud, cloud_ros);
        cloud_ros.header.frame_id = "map";  ///坐标系应该是当前submap 坐标系
        publisher.publish(cloud_ros);
    };
    pub_cloud(pub_laser_cloud_corner_from_map_DS_, corner_local_map);
    pub_cloud(pub_laser_cloud_surf_from_map_DS_ros_, surf_local_map);

    local_mapper_->FrameLocalMapper(corner_local_map, surf_local_map, ground_local_map, corner_cur_scan, surf_cur_scan,
                                    ground_cur_scan, opt_pose_kf_w);

    ///拍平
    if(fix_height_){
        auto fix_height = [](SE3& pose) {
          ///////四元数转欧拉角 开始
          auto q = pose.unit_quaternion();
          double sinr_cosp = +2.0 * (q.w() * q.x() + q.y() * q.z());
          double cosr_cosp = +1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
          double roll, pitch, yaw;
          roll = atan2(sinr_cosp, cosr_cosp);

          double sinp = +2.0 * (q.w() * q.y() - q.z() * q.x());
          if (fabs(sinp) >= 1) pitch = copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
          else
              pitch = asin(sinp);

          double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
          double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
          yaw = atan2(siny_cosp, cosy_cosp);
          ///////四元数转欧拉角 结束

          Mat33 n = (Eigen::AngleAxisd(0., Vec3::UnitX()) * Eigen::AngleAxisd(0., Vec3::UnitY()) *
              Eigen::AngleAxisd(yaw, Vec3::UnitZ()))
              .matrix();
          double x0 = pose.translation()[0];
          double y0 = pose.translation()[1];
          pose = SE3(n, Vec3(x0, y0, 1.32));
        };
        fix_height(opt_pose_kf_w);
    }

    cur_frame->SetPose(opt_pose_kf_w);
    last_frame_ = cur_frame;
}
}  // namespace lidar16
};  // namespace scrubber_slam
