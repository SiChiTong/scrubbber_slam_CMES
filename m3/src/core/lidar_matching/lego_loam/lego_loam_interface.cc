//
// Created by herenjie on 2021/5/10.
//

#include "lego_loam_interface.h"

namespace mapping::core {

LegoLoamInterface::LegoLoamInterface() {
//    ros::NodeHandle nh;
//    pub_laser_cloud_corner_from_map_DS_ = nh.advertise<sensor_msgs::PointCloud2>("/tracking/cloud_corner_map", 1);
//    pub_laser_cloud_surf_from_map_DS_ros_ = nh.advertise<sensor_msgs::PointCloud2>("/tracking/cloud_surfm_map", 1);
//    fa_ptr_ = std::make_shared<FeatureAssociation>(nh);
    fa_ptr_ = std::make_shared<FeatureAssociation>();
    local_map_ = std::make_shared<LocalMap>();
}

void LegoLoamInterface::SetStartID(int id) { start_id_ = id; }

void LegoLoamInterface::SetFirstFixedPose(const SE3& first_pose) { first_pose_ = first_pose; }

void LegoLoamInterface::SetInputCloud(const common::PointCloudType::Ptr& input, const int kf_id) {
    CloudPtr cur_cloud(new pcl::PointCloud<PointXYZI>());
    cur_cloud->clear();
    cur_cloud->resize(input->size());
    for (size_t i = 0; i < input->size(); i++) {
        cur_cloud->points[i].x = input->points[i].x;
        cur_cloud->points[i].y = input->points[i].y;
        cur_cloud->points[i].z = input->points[i].z;
        cur_cloud->points[i].intensity = input->points[i].intensity;
    }

    cur_frame_ = std::make_shared<lgFrame>(cur_cloud, kf_id);
}

void LegoLoamInterface::Run() {
    fa_ptr_->InsertLidar(cur_frame_->origin_cloud);
    fa_ptr_->GetSegCloud(cur_frame_->corner_points_sharp_, cur_frame_->corner_points_less_sharp_,
                         cur_frame_->surf_points_flat_, cur_frame_->surf_points_less_flat_);

    if (cur_frame_->id == start_id_) {
        cur_frame_->pose = first_pose_;
    } else {
        CloudPtr corner_local_map(new pcl::PointCloud<PointXYZI>);
        CloudPtr surf_local_map(new pcl::PointCloud<PointXYZI>);
        CloudPtr corner_cur_scan(new pcl::PointCloud<PointXYZI>);
        CloudPtr surf_cur_scan(new pcl::PointCloud<PointXYZI>);
        //    SE3 T_cur_last_ = pre_kf->dr_pose_.inverse() * cur_kf->dr_pose_;
        SE3 opt_pose_kf_w = last_frame_->pose * T_cur_last_;  ///预测世界系位姿

        *corner_cur_scan = *cur_frame_->corner_points_sharp_ + *cur_frame_->corner_points_less_sharp_;
        *surf_cur_scan = *cur_frame_->surf_points_flat_ + *cur_frame_->surf_points_less_flat_;

        LOG(INFO) << "local_map_corner_cloud_deque_: " << local_map_corner_cloud_deque_.size();
        for (auto& cloud : local_map_corner_cloud_deque_) {
            *corner_local_map += *cloud;
        }
        for (auto& cloud : local_map_surf_cloud_deque_) {
            *surf_local_map += *cloud;
        }
        LOG(INFO) << "local_map_surf_cloud_deque_: " << local_map_surf_cloud_deque_.size();

/*        auto pub_cloud = [](const ros::Publisher& publisher, const CloudPtr& cloud) {
            LOG(INFO) << "cloud: " << cloud->points.size();
            sensor_msgs::PointCloud2 cloud_ros;
            pcl::toROSMsg(*cloud, cloud_ros);
            cloud_ros.header.frame_id = "map";  ///坐标系应该是当前submap 坐标系
            publisher.publish(cloud_ros);
        };
        pub_cloud(pub_laser_cloud_corner_from_map_DS_, corner_local_map);
        pub_cloud(pub_laser_cloud_surf_from_map_DS_ros_, surf_local_map);*/

        local_map_->track(corner_local_map, surf_local_map, corner_cur_scan, surf_cur_scan, opt_pose_kf_w);
        cur_frame_->pose = opt_pose_kf_w;
    }
    matching_poses_.emplace(cur_frame_->id, cur_frame_->pose);
    V6d noise; noise << 0.05, 0.05, 0.05, 0.008, 0.008, 0.008;
    matching_noises_.emplace(cur_frame_->id, noise);
    AddFeatureDeque(cur_frame_);

    last_frame_ = cur_frame_;
}

void LegoLoamInterface::AddFeatureDeque(const std::shared_ptr<lgFrame>& cf) {
    ///关键帧特征点队列，用于局部地图匹配
    SE3& T_kf2w = cf->pose;
    CloudPtr corner_points_less_sharp(new pcl::PointCloud<PointXYZI>);
    pcl::transformPointCloud(*cf->corner_points_less_sharp_, *corner_points_less_sharp, T_kf2w.matrix());
    local_map_corner_cloud_deque_.push_back(corner_points_less_sharp);

    CloudPtr surf_points_less_flat(new pcl::PointCloud<PointXYZI>);
    pcl::transformPointCloud(*cf->surf_points_less_flat_, *surf_points_less_flat, T_kf2w.matrix());
    local_map_surf_cloud_deque_.push_back(surf_points_less_flat);

    if (local_map_surf_cloud_deque_.size() > 30) local_map_surf_cloud_deque_.pop_front();
    if (local_map_corner_cloud_deque_.size() > 30) local_map_corner_cloud_deque_.pop_front();

    //    cf->corner_points_sharp_->clear();cf->corner_points_sharp_= nullptr;
    //    cf->corner_points_less_sharp_->clear();cf->corner_points_less_sharp_= nullptr;
    //    cf->surf_points_flat_->clear();cf->surf_points_flat_= nullptr;
    //    cf->surf_points_less_flat_->clear();cf->surf_points_less_flat_= nullptr;
}

void LegoLoamInterface::SetPredictPose(const SE3& predict_pose) {
    //    SE3 predict_pose = pre_kf->dr_pose_.inverse() * cur_kf->dr_pose_;
    T_cur_last_ = predict_pose;
}

std::map<int, SE3> LegoLoamInterface::GetMatchingPoses() { return matching_poses_; }

std::map<int, V6d> LegoLoamInterface::GetMatchingNoise() { return matching_noises_; }

std::map<int, float> LegoLoamInterface::GetDegeneracyEigenValue() { return degeneracy_eigenvalue_; }

}  // namespace mapping::core
