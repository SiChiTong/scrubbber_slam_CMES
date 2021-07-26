//
// Created by gaoxiang on 2020/8/13.
//

#ifndef MAPPING_WGS84_COORDINATES_H
#define MAPPING_WGS84_COORDINATES_H

namespace mapping::common {

// WGS84坐标
struct WGS84Corr {
    WGS84Corr() = default;
    double lon = 0;  // longitude
    double lat = 0;  // latitude
};

}  // namespace mapping::common

#endif  // MAPPING_WGS84_COORDINATES_H
