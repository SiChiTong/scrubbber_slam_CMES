//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_DR_ELEMENTS_H
#define MAPPING_DR_ELEMENTS_H

#include "common/message_def.h"
#include "common/num_type.h"

namespace mapping::core {

struct DrElements {
    std_msgs::Header header;
    bool valid = true;
    double deltime = 0;
    V3d angular_velocity = V3d::Zero();
    V3d linear_acceleration = V3d::Zero();
    V3d position = V3d::Zero();
    V3d attitude = V3d::Zero();
    V3d velocity = V3d::Zero();
    double odom_reckoning_time = 0;
};

}  // namespace mapping::core

#endif  // MAPPING_DR_ELEMENTS_H
