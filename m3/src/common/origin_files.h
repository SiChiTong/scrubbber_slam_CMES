//
// Created by gaoxiang on 2020/8/12.
//

#ifndef MAPPING_ORIGIN_FILES_H
#define MAPPING_ORIGIN_FILES_H

#include <string>
#include <vector>

namespace mapping::common {

/// 原始数据文件信息，含完整的文件名
struct OriginFiles {
    void ClearFiles() {
        vehicle_params.clear();
        calib_params.clear();
        conf_params.clear();
        dbs_files.clear();
        pcd_files.clear();
        velodyne_params.clear();
        bag_files.clear();
        converted_bag_files.clear();
        bag_gps_status.clear();
    }

    std::vector<std::string> vehicle_params;       // 车辆参数
    std::vector<std::string> calib_params;         // 标定参数
    std::vector<std::string> conf_params;          // 配置参数
    std::vector<std::string> dbs_files;            // DB文件路径
    std::vector<std::string> pcd_files;            // PCD文件路径
    std::vector<std::string> velodyne_params;      // velodyne参数
    std::vector<std::string> bag_files;            // 各包的路径
    std::vector<std::string> converted_bag_files;  // 转换后各包的路径
    std::vector<int> bag_gps_status;               // 各包的GPS状态
};
}  // namespace mapping::common

#endif  // MAPPING_ORIGIN_FILES_H
