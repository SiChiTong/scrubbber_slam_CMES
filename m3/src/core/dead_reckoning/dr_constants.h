//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_DR_CONSTANTS_H
#define MAPPING_DR_CONSTANTS_H

namespace mapping::core {

const double kStaticTime = 1;  // 静止时间

const double fIMU_FREQ = 100;   // IMU频率
const double fGNSS_FREQ = 10;   // GPS频率
const double fODOM_FREQ = 10;   // Odom 频率
const double fLIDAR_FREQ = 10;  // Lidar 频率

const double tIMU_SPAN = 1.0 / fIMU_FREQ;
const double tGNSS_SPAN = 1.0 / fGNSS_FREQ;
const double tODOM_SPAN = 1.0 / fODOM_FREQ;
const double tLIDAR_SPAN = 1.0 / fLIDAR_FREQ;

const double WHEEL_RADIUS = 0.155;
const double CIRCLE_PULSE = 1024.0;

const double STATIC_VEL_THESHOLD = 0.005;
const double MOTION_VEL_THESHOLD = 2;

}  // namespace mapping::core

#endif  // MAPPING_DR_CONSTANTS_H
