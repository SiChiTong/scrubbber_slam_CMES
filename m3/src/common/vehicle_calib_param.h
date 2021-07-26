//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_VEHICLE_CALIB_PARAM_H
#define MAPPING_VEHICLE_CALIB_PARAM_H

namespace mapping::common {

/// 雷达参数
/// 在建图包中，雷达参数存储于calibration.launch中
/// 或者，在LADS中，由localization_map_collection.yaml指定标定文件
struct VehicleCalibrationParam {
    int lidar_type = 0;                      // 雷达线数，通常是16,32,64,80,128
    bool is_irad = false;                    // 是否是自研驱动
    std::string lidar_factory = "velodyne";  // 雷达生产厂家

    // 由感知标定的参数
    double perception_lidar_x_offset_top_center = 0;
    double perception_lidar_y_offset_top_center = 0;
    double perception_lidar_z_offset_top_center = 0;
    double perception_lidar_roll_top_center = 0;
    double perception_lidar_pitch_top_center = 0;
    double perception_lidar_yaw_top_center = 0;
    float static_gyro_var = 0.0;
    float static_acc_var = 0.0;
    float odom_ratio_left = 0;
    float odom_ratio_right = 0;
    float antenna_x = 0;
    float antenna_y = 0;
    float antenna_angle = 0;

    // 自研速腾激光驱动而添加
    int pre_rot_axis_0 = 0;
    int pre_rot_axis_1 = 1;
    int pre_rot_axis_2 = 2;
    double pre_rot_degree_0 = 0.0;
    double pre_rot_degree_1 = 0.0;
    double pre_rot_degree_2 = 0.0;
};

}  // namespace mapping::common

#endif  // MAPPING_VEHICLE_CALIB_PARAM_H
