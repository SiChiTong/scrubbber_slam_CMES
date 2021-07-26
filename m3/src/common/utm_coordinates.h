//
// Created by gaoxiang on 2020/8/13.
//

#ifndef MAPPING_UTM_COORDINATES_H
#define MAPPING_UTM_COORDINATES_H

namespace mapping {
namespace common {

// UTM坐标
struct UTMPoint {
    UTMPoint() {}
    double x = 0;
    double y = 0;
};

}  // namespace common
}  // namespace mapping

#endif  // MAPPING_UTM_COORDINATES_H
