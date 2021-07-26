//
// Created by idriver on 2020/10/15.
//

#ifndef MAPPING_SCAN_CONTEXT_H
#define MAPPING_SCAN_CONTEXT_H

#include <dirent.h>
#include <glog/logging.h>
#include "common/mapping_point_types.h"
#include "common/num_type.h"

#define CONTEXT_ROW 20
#define CONTEXT_COL 60
#define LMAX 80

namespace mapping::core {
/// scancontext初始化方法存储格式
struct sPose3f{
    float x;
    float y;
    float z;
    float roll;
    float pitch;
    float yaw;
};
struct ScanContextWithPose {
    sPose3f loc_pos;
    float context[CONTEXT_COL][CONTEXT_ROW];
};

class ScanContext {
   public:
    ScanContext();

    /// 构建单帧点云栅格数据并以bin文件形式存储起来
    void ConstructInitializationFile(const common::PointCloudXYZIT::ConstPtr &points_ptr, const SE3 &real_pose);

    /// 设置sacncontext文件路径
    void SetInitializationPath(const std::string initialization_path);

    /// 查找所有的scancontext初始化文件
    int FindSubFile(const std::string path);

    /// 获取指定位置的点云栅格数据以及对应的位姿数据
    inline core::ScanContextWithPose GetScanContextWithPose() { return context_and_pos_; };

   private:
    /// 构建单帧点云内容数据，即点云栅格化
    void ConstructScanContext(const common::PointCloudXYZIT::ConstPtr &points_ptr);

    /// 获取点云点所在的行列
    void GetColAndRow(const common::PointXYZIT &single_point, uint& col, uint& row);

    /// 保存scancontext初始化数据
    void SaveScanContextToFile(const ScanContextWithPose &context_and_pos);

    float resolution_col_ = 360 / CONTEXT_COL;   /// 6 degree
    float resolution_row_ = LMAX / CONTEXT_ROW;  /// 4m
    float context_[CONTEXT_COL][CONTEXT_ROW];
    ScanContextWithPose context_and_pos_;
    std::string sacncontext_path_;
    std::vector<std::string> files_;
};

}  // namespace mapping::core

#endif  // MAPPING_SCAN_CONTEXT_H
