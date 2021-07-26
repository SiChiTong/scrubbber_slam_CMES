//
// Created by idriver on 2020/10/15.
//

#ifndef MAPPING_SCAN_BUILDER_H
#define MAPPING_SCAN_BUILDER_H

#include <sys/stat.h>
#include "common/keyframe.h"
#include "common/mapping_point_types.h"
#include "common/num_type.h"
#include "core/scan_context/scan_context.h"
#include "io/file_io.h"

namespace mapping::pipeline {
class ScanBuilder {
   public:
    ScanBuilder(const std::string local_data_path, const std::string report);

    bool Init();

    int Start();

    /// 获取初始化文件信息
    inline std::vector<core::ScanContextWithPose> GetScanContextfiles() { return scan_context_files_; }

   private:
    void SaveScanContext(const common::PointCloudXYZIT::ConstPtr &input, SE3 &localizer);

    /// 点云转换
    void PointsCloudTF(const common::PointCloudType::ConstPtr &source_cloud, common::PointCloudXYZIT::Ptr &dest_cloud);

    bool ReadData(const std::string path);

    std::shared_ptr<core::ScanContext> scan_context_ = nullptr;
    std::vector<std::shared_ptr<common::KeyFrame>> pointcloud_vec_;
    std::string scan_context_path_;
    V3d last_loc_scan_;
    std::vector<core::ScanContextWithPose> scan_context_files_;
    bool using_scan_initial_ = false;
    std::string report_;
    std::string scan_file_path_;
};
}  // namespace mapping::pipeline

#endif  // MAPPING_SCAN_BUILDER_H
