//
// Created by idriver on 2020/10/15.
//

#include "scan_builder.h"
#include <boost/make_shared.hpp>

namespace mapping::pipeline {

ScanBuilder::ScanBuilder(const std::string local_data_path, const std::string report) {
    scan_file_path_ = local_data_path;
    LOG(INFO) << "scan_file_path_" << scan_file_path_;
    scan_context_path_ = scan_file_path_ + "scan_context/";
    report_ = report;
    scan_context_ = std::make_shared<core::ScanContext>();
    last_loc_scan_[0] = -1000;
    last_loc_scan_[1] = -1000;
}

bool ScanBuilder::Init() {
    try {
        if (access(scan_context_path_.c_str(), F_OK) != 0) {
            if (mkdir(scan_context_path_.c_str(), 0777) == -1) {
            }
        }
    } catch (...) {
        report_ += R"(\textcolor[rgb]{0.667,0.0,0.0}{Error : Some errors occur when creating scancontext file})";
        report_ += "\n";
        return false;
    }
    scan_context_->SetInitializationPath(scan_context_path_);
    LOG(INFO) << "scan_context_path_ :" << scan_context_path_;
    if (scan_context_->FindSubFile(scan_context_path_) > 0) {
        using_scan_initial_ = true;
    } else {
        using_scan_initial_ = false;
    }

    return true;
}

int ScanBuilder::Start() {
    ReadData(scan_file_path_ + "map.db");

    common::PointCloudXYZIT::Ptr points_cloud = boost::make_shared<common::PointCloudXYZIT>();

    if (pointcloud_vec_.empty()) {
        LOG(INFO) << "pointcloud_vec_ is empty";
    }

    for (auto &kf : pointcloud_vec_) {
        points_cloud->clear();

        PointsCloudTF(kf->cloud_, points_cloud);

        if (points_cloud != nullptr) {
            SaveScanContext(points_cloud, kf->optimized_pose_stage_2_);
        } else {
            LOG(INFO) << "points_cloud is null";
        }
    }

    LOG(INFO) << "scan_context_files_.size():" << scan_context_files_.size();

    return 0;
}

void ScanBuilder::SaveScanContext(const common::PointCloudXYZIT::ConstPtr &input, SE3 &localizer) {
    scan_context_->ConstructInitializationFile(input, localizer);
    scan_context_files_.push_back(scan_context_->GetScanContextWithPose());

    return;
}

bool ScanBuilder::ReadData(const std::string path) {
    std::vector<std::shared_ptr<common::KeyFrame>>().swap(pointcloud_vec_);

    if (!io::KeyFramesPoints(path, pointcloud_vec_)) {
        LOG(ERROR) << "read keyframe points db file failed : " << path;
        return false;
    }

    return true;
}

void ScanBuilder::PointsCloudTF(const common::PointCloudType::ConstPtr &source_cloud,
                                common::PointCloudXYZIT::Ptr &dest_cloud) {
    if (source_cloud->points.size() <= 0) {
        return;
    }
    for (const auto &point : source_cloud->points) {
        common::PointXYZIT new_point;
        new_point.x = point.x;
        new_point.y = point.y;
        new_point.z = point.z;
        new_point.intensity = point.intensity;
        dest_cloud->points.push_back(new_point);
    }
    dest_cloud->header.stamp = source_cloud->header.stamp;
    dest_cloud->height = 1;
    dest_cloud->width = dest_cloud->points.size();

    return;
}

}  // namespace mapping::pipeline
