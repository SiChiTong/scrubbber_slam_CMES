//
// Created by gaoxiang on 2020/9/21.
//

#include "common/keyframe.h"

namespace mapping::common {

void KeyFrame::SetGpsNoiseFromStatus() {
    V3d pos_noise;
    switch (gps_status_) {
        case GpsStatusType::GNSS_FIXED_SOLUTION:
            pos_noise << gnss_fixed_noise, gnss_fixed_noise, gnss_fixed_noise;
            break;
        case GpsStatusType::GNSS_FLOAT_SOLUTION:
            pos_noise << gnss_float_noise, gnss_float_noise, gnss_float_noise;
            break;
        case GpsStatusType::GNSS_PSEUDO_SOLUTION:
            pos_noise << gnss_pseudo_noise, gnss_pseudo_noise, gnss_pseudo_noise;
            break;
        case GpsStatusType::GNSS_SINGLE_POINT_SOLUTION:
            pos_noise << gnss_single_point_noise, gnss_single_point_noise, gnss_single_point_noise;
            break;
        case GpsStatusType::GNSS_NOT_EXIST:
        case GpsStatusType::GNSS_OTHER:
            pos_noise << gnss_not_exist_noise, gnss_not_exist_noise, gnss_not_exist_noise;
            break;
        default:
            break;
    }

    gps_noise_.head<3>() = pos_noise;
    if (heading_valid_) {
        gps_noise_.tail<3>() = V3d(0.08, 0.08, 0.08);  // 5度
    } else {
        gps_noise_.tail<3>() = V3d(1e6, 1e6, 1e6);
    }
}

}  // namespace mapping::common