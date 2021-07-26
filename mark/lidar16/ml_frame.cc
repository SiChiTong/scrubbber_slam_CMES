//
// Created by gaoxiang on 2020/10/29.
//

#include "lidar16/ml_frame.h"

#include <utility>

namespace scrubber_slam { namespace lidar16 {
Idtype s_ml_frame_id = 0;

//MLFrame::MLFrame(){
//    cloud_ptr_ = boost::make_shared<PointCloudType>();
//}

void MLFrame::SetIdcountZero(){
//            s_ml_frame_id_ = 0;
    s_ml_frame_id = 0;
}

std::shared_ptr<MLFrame> MLFrame::CreateMLFrame(CloudPtr cloud) {
    auto newframe = std::make_shared<MLFrame>();
    newframe->id_ = s_ml_frame_id++;
//    newframe->id_ = s_ml_frame_id_++;
    newframe->cloud_ptr_ = std::move(cloud);

    newframe->corner_points_sharp_ = boost::make_shared<PointCloudType>();
    newframe->corner_points_less_sharp_ = boost::make_shared<PointCloudType>();
    newframe->surf_points_flat_ = boost::make_shared<PointCloudType>();
    newframe->surf_points_less_flat_ = boost::make_shared<PointCloudType>();
    newframe->ground_cloud_ = boost::make_shared<PointCloudType>();
    return newframe;
}

std::shared_ptr<MLFrame> MLFrame::CreateMLFrame(const mapping::common::PointCloudPtr& cloudXYZIHIRBS) {
    auto newframe = std::make_shared<MLFrame>();
    newframe->id_ = s_ml_frame_id++;
//    newframe->id_ = s_ml_frame_id_++;
    newframe->cloudXYZIHIRBS_ptr_ = cloudXYZIHIRBS;

    CloudPtr cloud(new PointCloudType);
    cloud->resize(cloudXYZIHIRBS->points.size());
    for(int i=0;i<cloudXYZIHIRBS->points.size();i++){
        cloud->points[i].x = cloudXYZIHIRBS->points[i].x;
        cloud->points[i].y = cloudXYZIHIRBS->points[i].y;
        cloud->points[i].z = cloudXYZIHIRBS->points[i].z;
        cloud->points[i].intensity = cloudXYZIHIRBS->points[i].intensity;
    }
    newframe->cloud_ptr_ = std::move(cloud);

    newframe->corner_points_sharp_ = boost::make_shared<PointCloudType>();
    newframe->corner_points_less_sharp_ = boost::make_shared<PointCloudType>();
    newframe->surf_points_flat_ = boost::make_shared<PointCloudType>();
    newframe->surf_points_less_flat_ = boost::make_shared<PointCloudType>();
    newframe->ground_cloud_ = boost::make_shared<PointCloudType>();

    return newframe;
}

} }  // namespace scrubber_slam::lidar16