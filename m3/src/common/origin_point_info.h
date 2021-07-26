//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_ORIGIN_POINT_INFO_H
#define MAPPING_ORIGIN_POINT_INFO_H

namespace mapping::common {

/// 地图原点信息
struct OriginPointInformation {
    double map_origin_x = 0;   // 原点X
    double map_origin_y = 0;   // 原点Y
    double map_origin_z = 0;   // 原点Z
    int map_origin_zone = 50;  // 时区
    bool is_southern = false;  // 是否在南半球
};

}  // namespace mapping::common

#endif  // MAPPING_ORIGIN_POINT_INFO_H
