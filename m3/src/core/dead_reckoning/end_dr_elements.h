//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_END_DR_ELEMENTS_H
#define MAPPING_END_DR_ELEMENTS_H

#include "common/message_def.h"
#include "common/num_type.h"

#include "core/dead_reckoning/average_elements.h"
#include "core/dead_reckoning/dr_elements.h"
#include "core/dead_reckoning/imu_elements.h"
#include "core/dead_reckoning/imu_variance_elements.h"
#include "core/dead_reckoning/odom_elements.h"

namespace mapping::core {

struct EndDrElements {
    V6d States;
    M6d P;
    M6d IKH;
    M6d F;
    M6d G;
    M6d Q;
    M6d Qk;
    H6d H;
    H1d R;

    AverageElements imu_avge;
    DrElements dr_data;
    ImuElements imu_data;
    OdomElements odom_data;

    unsigned int integral_odom[3];
    unsigned int clk_system;
    unsigned int imu_time_ms_psd;

    double imu_time_ros_psd;
    double odom_time_ros_psd;
    double velocity_odom_psd;
    double delta_time_ms;

    V3d vel_psd;

    std::deque<DrElements> dr_deque;
    std::deque<ImuElements> imu_data_deque;

    ImuVarianceElements imu_variance;
};

}  // namespace mapping::core

#endif  // MAPPING_END_DR_ELEMENTS_H
