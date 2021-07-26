//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_IMU_VARIANCE_ELEMENTS_H
#define MAPPING_IMU_VARIANCE_ELEMENTS_H

#include "common/mapping_math.h"
#include "common/message_def.h"
#include "common/num_type.h"
#include "core/dead_reckoning/imu_elements.h"

#include <deque>

namespace mapping::core {

struct ImuVarianceElements {
    double time = 0;
    V3d gyro_mean = V3d::Ones() * 100;
    V3d acce_mean = V3d::Ones() * 100;
    V3d gyro_var = V3d::Ones() * 100;
    V3d acce_var = V3d::Ones() * 100;
    double gyro_var_norm = 100;
    double acce_var_norm = 100;

    void GetMsg(std::deque<ImuElements> imu_deque) {
        V3d accum_gyro;
        accum_gyro.setZero();
        V3d accum_acce;
        accum_acce.setZero();
        double accum_time = 0.0;
        int len = imu_deque.size();
        if (len < 10) {
            return;
        }

        for (int k = 0; k < len; k++) {
            accum_time += imu_deque[k].header.stamp.toSec();
            accum_gyro += imu_deque[k].angular_velocity;
            accum_acce += imu_deque[k].linear_acceleration;
        }

        time = accum_time / len;
        gyro_mean = accum_gyro / len;
        acce_mean = accum_acce / len;

        gyro_var.setZero();
        acce_var.setZero();

        for (int k = 0; k < len; k++) {
            V3d gyro_dev = imu_deque[k].angular_velocity - gyro_mean;
            V3d acce_dev = imu_deque[k].linear_acceleration - acce_mean;

            for (int i = 0; i < 3; i++) {
                gyro_var[i] += common::SQ(gyro_dev[i]);
                acce_var[i] += common::SQ(acce_dev[i]);
            }
        }

        gyro_var /= (len - 1);
        acce_var /= (len - 1);

        gyro_var_norm = gyro_var.norm();
        acce_var_norm = acce_var.norm();
    }
};
}  // namespace mapping::core

#endif  // MAPPING_IMU_VARIANCE_ELEMENTS_H
