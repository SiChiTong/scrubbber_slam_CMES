//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_IMU_ELEMENTS_H
#define MAPPING_IMU_ELEMENTS_H

#include "common/message_def.h"
#include "common/num_type.h"

namespace mapping::core {

struct ImuElements {
    std_msgs::Header header;
    bool update = false;
    unsigned int time_ms = 0;
    double deltime_ms = 0;
    double deltime_ros = 0;
    V3d angular_velocity = V3d::Zero();
    V3d linear_acceleration = V3d::Zero();
    V3d gyro_bias = V3d::Zero();
    V3d acce_bias = V3d::Zero();

    void GetMsg(const ImuMsg &msg) {
        header.stamp = msg.header.stamp;
        update = true;
        time_ms = msg.TimeTag;
        angular_velocity(0) = msg.gyro_x;
        angular_velocity(1) = msg.gyro_y;
        angular_velocity(2) = msg.gyro_z;
        linear_acceleration(0) = msg.acce_x;
        linear_acceleration(1) = msg.acce_y;  // unit m/s^2
        linear_acceleration(2) = msg.acce_z;
    }
};

}  // namespace mapping::core

#endif  // MAPPING_IMU_ELEMENTS_H
