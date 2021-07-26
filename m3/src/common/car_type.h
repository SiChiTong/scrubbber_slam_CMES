//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_CAR_TYPE_H
#define MAPPING_CAR_TYPE_H

namespace mapping::common {

/// 车辆类型定义
enum class CarType {
    WXX = 0,  // 蜗小白或蜗必达
    LADS,     // LADS建图
    OTHERS,   // 其他类型车辆
};

}  // namespace mapping::common

#endif  // MAPPING_CAR_TYPE_H
