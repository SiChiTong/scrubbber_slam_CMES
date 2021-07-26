//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_TOPICS_DEF_H
#define MAPPING_TOPICS_DEF_H

#include <string>

namespace mapping::common::topics {

const std::string tp_image = "/camera/image0/compressed";
const std::string tp_gps = "/ivsensorgps";
const std::string tp_imu = "/ivsensorimu";
const std::string tp_wheelspeed = "/ivwheelspeed";
const std::string tp_cansensor = "/tpcansensor";
const std::string tp_appwxb = "/specialfunctionpoint";
const std::string tp_appwbd = "/app2taskcentral";
const std::string tp_lidar = "/velodyne_packets_1";
const std::string tp_lidarpose = "/ndt_status";
const std::string tp_localpose = "/tplocalization";

/// velodyne lads
#ifdef _VELODYNE_64
const std::string tp_lidar_velodyne_lads = "/velodyne_packets_2"; // velodyne_64
#else
const std::string tp_lidar_velodyne_lads = "/driver/lidar/vlp_32_packets/center";
#endif
/// 禾赛
const std::string tp_lidar_hesai = "/lidar_packets_2";

/// 速腾
const std::string tp_lidar_suteng_scan = "/driver/lidar/rs_80_packets/top_center";
const std::string tp_lidar_suteng_packets = "/driver/lidar/rs_80_packets/top_center_difop";

}  // namespace mapping::common::topics

#endif  // MAPPING_TOPICS_DEF_H
