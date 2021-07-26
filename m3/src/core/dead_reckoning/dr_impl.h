//
// Created by gaoxiang on 2020/8/14.
//

#ifndef MAPPING_DR_IMPL_H
#define MAPPING_DR_IMPL_H

#include "common/num_type.h"
#include "common/std_headers.h"

#include "core/dead_reckoning/average_elements.h"
#include "core/dead_reckoning/dr_elements.h"
#include "core/dead_reckoning/dr_params.h"
#include "core/dead_reckoning/imu_elements.h"
#include "core/dead_reckoning/imu_variance_elements.h"
#include "core/dead_reckoning/odom_elements.h"

namespace mapping::core {

struct DRImpl {
    DRImpl() : dr_deque_size_(200), imu_deque_max_size_(50) {}

    DrParams dr_params_;
    Eigen::Matrix<double, 6, 1> States_;
    Eigen::Matrix<double, 6, 6> P_;
    Eigen::Matrix<double, 6, 6> IKH_;
    Eigen::Matrix<double, 6, 6> F_;
    Eigen::Matrix<double, 6, 6> G_;
    Eigen::Matrix<double, 6, 6> Q_;
    Eigen::Matrix<double, 6, 6> Qk_;
    Eigen::Matrix<double, 1, 6> H_;
    Eigen::Matrix<double, 1, 1> R_;
    double odom_vel_noise_{};
    double lambda_ = 0.0;
    double vel_max_lambda_ = 50.0;

    ImuElements imu_data_;
    OdomElements odom_data_;
    AverageElements imu_avge_;
    DrElements dr_data_;

    std::deque<DrElements> dr_deque_;
    size_t dr_deque_size_;

    Quat quaternion_;

    double latitude_{};
    unsigned int integral_odom[3]{};
    unsigned int imu_time_ms_psd_{};
    double imu_time_ros_psd_{};
    double odom_time_ros_psd_{};
    double gnss_time_ros_psd_{};
    double lidar_time_ros_psd_{};

    double odom_velocity_l_{};
    double odom_velocity_r_{};

    bool horizontal_initial_flag_{};
    bool gyroscope_initial_flag_{};
    bool azimuth_initial_flag_{};
    bool system_initial_flag_{};
    bool position_initial_flag_{};
    bool reset_horizontal_flag_{};

    unsigned int clk_system_{};
    double velocity_odom_psd_{};
    double delta_time_ms_{};
    V3d vel_psd_;

    // imu data deque
    std::deque<ImuElements> imu_data_deque_;
    size_t imu_deque_max_size_;
    ImuVarianceElements imu_variance_;
};

}  // namespace mapping::core

#endif  // MAPPING_DR_IMPL_H
