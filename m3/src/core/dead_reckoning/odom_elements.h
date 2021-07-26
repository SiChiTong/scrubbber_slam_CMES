//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_ODOM_ELEMENTS_H
#define MAPPING_ODOM_ELEMENTS_H

#include "common/constants.h"
#include "common/message_def.h"
#include "common/num_type.h"
#include "core/dead_reckoning/dr_constants.h"

namespace mapping::core {

struct OdomElements {
    std_msgs::Header header;
    bool update = false;
    bool initial = false;
    double deltime_ros = 0;
    double angular_l = 0, angular_r = 0;
    double velocity_l = 0, velocity_r = 0;
    double velocity_filter = 0;
    V3d velocity_add = V3d::Zero();
    V3d vel_xyz = V3d::Zero();
    V3d velocity = V3d::Zero();
    V3d position = V3d::Zero();
    V3d position_inf = V3d::Zero();
    double yaw_rate = 0;
    double coeff = 0;

    OdomElements() {
        update = false;
        initial = false;
        deltime_ros = 0.0;
        angular_l = angular_r = 0.0;
        velocity_l = velocity_r = 0.0;
        velocity_filter = 0.0;
        yaw_rate = 0.0;
        velocity_add.setZero();
        vel_xyz.setZero();
        velocity.setZero();
        position.setZero();
        position_inf.setZero();
        coeff = 0.75;
    }

    void GetMsg(const OdomMsg& msg, const M3d& cbn, double rl, double rr, double factor, double width_base) {
        header.stamp = msg.header.stamp;
        update = true;
        // get odom velocity
        velocity_l = (WHEEL_RADIUS * rl) * msg.wheelspeed_lr_pluse / CIRCLE_PULSE * 2 * common::kPI / tODOM_SPAN;
        velocity_r = (WHEEL_RADIUS * rr) * msg.wheelspeed_rr_pluse / CIRCLE_PULSE * 2 * common::kPI / tODOM_SPAN;
        vel_xyz(1) = 0.5 * (velocity_l + velocity_r) * factor;
        velocity = cbn * vel_xyz;

        // lowpass filter
        angular_l =
            coeff * angular_l + (1.0 - coeff) * msg.wheelspeed_lr_pluse / CIRCLE_PULSE * 2 * common::kPI / tODOM_SPAN;
        angular_r =
            coeff * angular_r + (1.0 - coeff) * msg.wheelspeed_rr_pluse / CIRCLE_PULSE * 2 * common::kPI / tODOM_SPAN;

        velocity_filter = 0.5 * (angular_l * rl + angular_r * rr) * WHEEL_RADIUS * factor;

        // yaw rate from odom velocity(rad/s)
        yaw_rate = WHEEL_RADIUS * (angular_r * rr - angular_l * rl) / width_base;
    }

    void SetPos(const V3d& pos) {
        initial = true;
        position = pos;
    }

    void UpdatePos(const M3d& cbn, double tau) {
        if (initial) {
            V3d vel_enu;
            vel_enu = cbn * vel_xyz;
            position += vel_enu * tau;
            position_inf += vel_enu * tau;
        }
    }

    void CompeArm(const V3d& laram, const V3d& angular, const M3d& cbn) {
        velocity_add = cbn * angular.cross(laram);
        velocity += velocity_add;
    }
};

}  // namespace mapping::core

#endif  // MAPPING_ODOM_ELEMENTS_H
