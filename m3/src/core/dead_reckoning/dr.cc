//
// Created by gaoxiang on 2020/8/14.
//
#include "core/dead_reckoning/dr.h"
#include "common/constants.h"
#include "core/dead_reckoning/dr_constants.h"
#include "core/dead_reckoning/dr_impl.h"

#include <glog/logging.h>
#include <utility>

using namespace mapping::common;

namespace mapping::core {

DeadReckoning::DeadReckoning() : impl_(new DRImpl()) { Initialization(); }

DeadReckoning::DeadReckoning(const DrParams &dr_params) : impl_(new DRImpl()) {
    impl_->dr_params_ = dr_params;
    Initialization();
}

void DeadReckoning::Initialization() {
    impl_->latitude_ = 40;
    impl_->imu_time_ms_psd_ = 0;
    impl_->imu_time_ros_psd_ = 0.0;
    impl_->odom_time_ros_psd_ = 0.0;
    impl_->gnss_time_ros_psd_ = 0.0;
    impl_->lidar_time_ros_psd_ = 0.0;
    impl_->odom_velocity_l_ = 5.0;
    impl_->odom_velocity_r_ = 5.0;
    impl_->horizontal_initial_flag_ = false;
    impl_->gyroscope_initial_flag_ = false;
    impl_->azimuth_initial_flag_ = false;
    impl_->system_initial_flag_ = false;
    impl_->position_initial_flag_ = false;
    impl_->reset_horizontal_flag_ = false;
    impl_->clk_system_ = 0;
    impl_->velocity_odom_psd_ = 0;
    impl_->vel_psd_.setZero();
    impl_->imu_avge_.InitAvg(kStaticTime);
    impl_->odom_vel_noise_ = 0.2;
    impl_->integral_odom[0] = impl_->integral_odom[1] = impl_->integral_odom[2] = 1000;

    impl_->P_.setZero();
    impl_->F_.setZero();
    impl_->G_.setZero();
    impl_->Q_.setZero();
    impl_->Qk_.setZero();
    impl_->H_.setZero();

    impl_->States_.setZero();

    // initial cov matrix
    impl_->P_(0, 0) = impl_->P_(1, 1) = impl_->P_(2, 2) = SQ(0.5 * kDEG2RAD);
    impl_->P_(3, 3) = impl_->P_(4, 4) = impl_->P_(5, 5) = SQ(0.5);

    impl_->Q_(0, 0) = impl_->Q_(1, 1) = impl_->Q_(2, 2) = SQ(0.1 * kDEG2RAD);
    impl_->Q_(3, 3) = impl_->Q_(4, 4) = impl_->Q_(5, 5) = SQ(30 * kM_GRAVITY);
}

bool DeadReckoning::SetImuElements(const ImuMsg &msg) {
    impl_->imu_data_.GetMsg(msg);

    double current_time_ms = msg.TimeTag;
    impl_->delta_time_ms_ = current_time_ms - impl_->imu_time_ms_psd_;
    if (impl_->delta_time_ms_ < 0) impl_->delta_time_ms_ += 655360;
    impl_->imu_data_.deltime_ms = impl_->delta_time_ms_ / 1000.0;
    impl_->imu_time_ms_psd_ = current_time_ms;

    double current_time_ros = msg.header.stamp.toSec();
    impl_->imu_data_.deltime_ros = current_time_ros - impl_->imu_time_ros_psd_;
    impl_->imu_time_ros_psd_ = current_time_ros;

    impl_->integral_odom[0]++;
    impl_->integral_odom[1]++;
    impl_->integral_odom[2]++;

    CacheImuData(impl_->imu_data_);

    if (impl_->imu_data_deque_.size() > (impl_->imu_deque_max_size_ - 10)) {
        impl_->imu_variance_.GetMsg(impl_->imu_data_deque_);
    }

    if (impl_->imu_data_.deltime_ms < 0.001) {
        impl_->imu_data_.deltime_ms = 0.01;
    }
    if (impl_->imu_data_.deltime_ms > 0.05) {
        impl_->imu_data_.deltime_ms = 0.01;
    }
    if (impl_->imu_data_.deltime_ros > 0.05) {
        impl_->imu_data_.deltime_ros = 0.01;
    }
    if (impl_->imu_data_.deltime_ros < 1e-5) {
        impl_->imu_data_.deltime_ros = 0.01;
    }

    return true;
}

bool DeadReckoning::SetOdomElements(const OdomMsg &msg) {
    M3d cbn;
    cbn.setZero();
    cbn(0, 0) = cbn(1, 1) = cbn(2, 2) = 1.0;

    impl_->odom_data_.GetMsg(msg, cbn, impl_->dr_params_.ratio_left, impl_->dr_params_.ratio_right, 1.0, 0.749);

    double current_time_ros = msg.header.stamp.toSec();
    double deltime_ros = current_time_ros - impl_->odom_time_ros_psd_;
    impl_->odom_data_.deltime_ros = deltime_ros;
    impl_->odom_time_ros_psd_ = current_time_ros;

    if (impl_->odom_data_.deltime_ros > 1) impl_->odom_data_.deltime_ros = 0.1;
    if (impl_->odom_data_.deltime_ros < 0.001) impl_->odom_data_.deltime_ros = 0.1;

    impl_->odom_velocity_l_ = msg.wheelspeed_lr_pluse / 1024.0 * 2 * kPI * impl_->dr_params_.frequence;
    impl_->odom_velocity_r_ = msg.wheelspeed_rr_pluse / 1024.0 * 2 * kPI * impl_->dr_params_.frequence;

    if (impl_->dr_deque_.size() < impl_->dr_deque_size_) {
        return false;
    }

    size_t p_index = GetSuitableIndex(current_time_ros);

    V3d attitude = impl_->dr_deque_[p_index].attitude;
    Att2Cbn(cbn, attitude);
    impl_->odom_data_.velocity = cbn * impl_->odom_data_.vel_xyz;

    V3d innov = impl_->dr_deque_[p_index].velocity - impl_->odom_data_.velocity;

    Correct(innov, impl_->dr_data_.velocity);

    return true;
}

void DeadReckoning::SetPosition(V3d position) {
    impl_->dr_data_.position = std::move(position);
    impl_->position_initial_flag_ = true;
}

void DeadReckoning::SetAzimuth(double azimuth) {
    impl_->dr_data_.attitude(2) = azimuth * kDEG2RAD;
    impl_->quaternion_ = Att2Quat(impl_->dr_data_.attitude);
    impl_->azimuth_initial_flag_ = true;

    //    horizontal_initial_flag_ = true;
}

void DeadReckoning::SetInitalFlag() {
    impl_->position_initial_flag_ = true;
    impl_->azimuth_initial_flag_ = true;
    impl_->reset_horizontal_flag_ = false;
    impl_->horizontal_initial_flag_ = true;
    impl_->gyroscope_initial_flag_ = true;
}

void DeadReckoning::ApplyDeadReckoning() {
    impl_->clk_system_++;
    InitialSystem();
    if ((impl_->horizontal_initial_flag_ && impl_->azimuth_initial_flag_ && impl_->position_initial_flag_) ||
        (impl_->system_initial_flag_)) {
        if (impl_->imu_data_.update) {
            impl_->imu_data_.update = false;
            impl_->system_initial_flag_ = true;
            SetDrElements();
            QuaternionUpdate(impl_->dr_data_.angular_velocity, impl_->dr_data_.velocity, impl_->dr_data_.deltime);
            VelocityUpdate(impl_->dr_data_.velocity, impl_->dr_data_.linear_acceleration, impl_->dr_data_.deltime);
            PositionUpdate(impl_->dr_data_.position, impl_->odom_data_.velocity, impl_->dr_data_.deltime);
            Predict(impl_->dr_data_.linear_acceleration, impl_->dr_data_.deltime);
            impl_->dr_data_.attitude = Quat2Att(impl_->quaternion_);
            CacheDrData(impl_->dr_data_);
        }
    }
}

void DeadReckoning::Predict(const V3d &fibb, double tau) {
    M3d cbn = impl_->quaternion_.toRotationMatrix();
    V3d fibn = cbn * fibb;

    impl_->F_.setZero();
    impl_->F_(0, 0) = impl_->F_(1, 1) = impl_->F_(2, 2) = 1.0;
    impl_->F_(3, 3) = impl_->F_(4, 4) = impl_->F_(5, 5) = 1.0;

    impl_->F_(3, 1) = -fibn(2) * tau;
    impl_->F_(3, 2) = fibn(1) * tau;
    impl_->F_(4, 0) = fibn(2) * tau;
    impl_->F_(4, 2) = -fibn(0) * tau;
    impl_->F_(5, 0) = -fibn(1) * tau;
    impl_->F_(5, 1) = fibn(0) * tau;

    impl_->G_(0, 0) = -cbn(0, 0);
    impl_->G_(0, 1) = -cbn(0, 1);
    impl_->G_(0, 2) = -cbn(0, 2);
    impl_->G_(1, 0) = -cbn(1, 0);
    impl_->G_(1, 1) = -cbn(1, 1);
    impl_->G_(1, 2) = -cbn(1, 2);
    impl_->G_(2, 0) = -cbn(2, 0);
    impl_->G_(2, 1) = -cbn(2, 1);
    impl_->G_(2, 2) = -cbn(2, 2);

    impl_->G_(3, 3) = cbn(0, 0);
    impl_->G_(3, 4) = cbn(0, 1);
    impl_->G_(3, 5) = cbn(0, 2);
    impl_->G_(4, 3) = cbn(1, 0);
    impl_->G_(4, 4) = cbn(1, 1);
    impl_->G_(4, 5) = cbn(1, 2);
    impl_->G_(5, 3) = cbn(2, 0);
    impl_->G_(5, 4) = cbn(2, 1);
    impl_->G_(5, 5) = cbn(2, 2);

    impl_->Qk_ = impl_->G_ * impl_->Q_ * impl_->G_.transpose() * tau;

    impl_->States_ = impl_->F_ * impl_->States_;

    impl_->P_ = impl_->F_ * impl_->P_ * impl_->F_.transpose() + impl_->Qk_;
}

void DeadReckoning::Correct(V3d innov, V3d &vel) {
    Eigen::Matrix<double, 1, 1> y_;
    Eigen::Matrix<double, 1, 1> S_;
    Eigen::Matrix<double, 6, 1> K_;

    impl_->R_(0) = SQ(impl_->odom_vel_noise_);

    for (int k = 0; k < 3; k++) {
        impl_->H_.setZero();
        impl_->H_(0, k + 3) = 1.0;
        y_(0) = innov(k) - impl_->H_ * impl_->States_;
        S_ = impl_->H_ * impl_->P_ * impl_->H_.transpose() + impl_->R_;
        double ss = S_(0);
        K_ = impl_->P_ * impl_->H_.transpose() / ss;
        impl_->lambda_ = y_(0) * y_(0) / ss;

        if (impl_->lambda_ < impl_->vel_max_lambda_) {
            impl_->States_ = impl_->States_ + K_ * y_;
            impl_->IKH_ = Eigen::Matrix<double, 6, 6>::Identity() - K_ * impl_->H_;
            impl_->P_ = impl_->IKH_ * impl_->P_ * impl_->IKH_.transpose() + K_ * impl_->R_ * K_.transpose();

            impl_->integral_odom[k] = 0;
        }
    }

    V3d attError;
    attError(0) = impl_->States_(0);
    attError(1) = impl_->States_(1);
    attError(2) = impl_->States_(2);
    QuatCorrect(attError);

    vel(0) -= impl_->States_(3);
    vel(1) -= impl_->States_(4);
    vel(2) -= impl_->States_(5);

    impl_->States_.setZero();
}

void DeadReckoning::QuatCorrect(const V3d &attError) {
    M3d Cbn_cur, Cbn, Rot;
    Cbn = impl_->quaternion_.toRotationMatrix();
    // Rot = I + skew(err)
    Rot(0, 0) = Rot(1, 1) = Rot(2, 2) = 1.0;
    Rot(0, 1) = -attError(2);
    Rot(1, 0) = -Rot(0, 1);
    Rot(0, 2) = attError(1);
    Rot(2, 0) = -Rot(0, 2);
    Rot(1, 2) = -attError(0);
    Rot(2, 1) = -Rot(1, 2);
    Cbn_cur = Rot * Cbn;
    impl_->quaternion_ = Quat(Cbn_cur);
}

void DeadReckoning::CacheDrData(const DrElements &msg) {
    impl_->dr_deque_.push_back(msg);
    size_t size = impl_->dr_deque_.size();
    while (size > impl_->dr_deque_size_) {
        impl_->dr_deque_.pop_front();
        --size;
    }
}

void DeadReckoning::CacheImuData(const ImuElements &msg) {
    impl_->imu_data_deque_.push_back(msg);
    size_t size = impl_->imu_data_deque_.size();
    while (size > impl_->imu_deque_max_size_) {
        impl_->imu_data_deque_.pop_front();
        --size;
    }
}

size_t DeadReckoning::GetSuitableIndex(double timetic) {
    size_t len = impl_->dr_deque_.size();
    size_t pIndex = len;
    for (int k = len - 1; k >= 0; --k) {
        double Delay_t = impl_->dr_deque_[k].header.stamp.toSec() - timetic;

        if (Delay_t <= 0) {
            pIndex = k;
            break;
        }
    }
    return pIndex;
}

void DeadReckoning::SetDrElements() {
    impl_->dr_data_.header.stamp = impl_->imu_data_.header.stamp;
    impl_->dr_data_.deltime = impl_->imu_data_.deltime_ms;
    impl_->dr_data_.angular_velocity = (impl_->imu_data_.angular_velocity - impl_->imu_data_.gyro_bias) * kDEG2RAD;
    impl_->dr_data_.linear_acceleration = impl_->imu_data_.linear_acceleration - impl_->imu_data_.acce_bias;
}

bool DeadReckoning::StaticCheck(double x, double y) {
    return fabs(x) <= STATIC_VEL_THESHOLD && fabs(y) <= STATIC_VEL_THESHOLD;
}

void DeadReckoning::InitialSystem() {
    if (StaticCheck(impl_->odom_velocity_l_, impl_->odom_velocity_r_)) {
        if (impl_->imu_variance_.gyro_var_norm < impl_->dr_params_.static_gyro_var &&  // static gyro_var
            impl_->imu_variance_.acce_var_norm < impl_->dr_params_.static_acc_var) {
            if (impl_->imu_data_.update) {
                V3d gyro = impl_->imu_data_.angular_velocity;
                V3d acce = impl_->imu_data_.linear_acceleration - impl_->imu_data_.acce_bias;

                if (1 == impl_->imu_avge_.GetAvg(gyro, acce, impl_->imu_data_.deltime_ros)) {
                    impl_->imu_avge_.Status = 0;

                    if (!impl_->gyroscope_initial_flag_) {
                        impl_->imu_data_.gyro_bias = impl_->imu_avge_.avg1;
                        impl_->gyroscope_initial_flag_ = false;
                    }

                    if (!impl_->horizontal_initial_flag_ || impl_->reset_horizontal_flag_) {
                        impl_->reset_horizontal_flag_ = false;
                        impl_->horizontal_initial_flag_ = true;
                        impl_->dr_data_.attitude(0) = atan2(-impl_->imu_avge_.avg2(0), impl_->imu_avge_.avg2(2));
                        impl_->dr_data_.attitude(1) = asin(impl_->imu_avge_.avg2(1) / kGRAVITY);
                        impl_->quaternion_ = Att2Quat(impl_->dr_data_.attitude);
                    }
                }
            }
        } else {
            static double resting_time = 0;
            resting_time += impl_->imu_data_.deltime_ms;

            if (resting_time > 119 && impl_->imu_variance_.gyro_var_norm < 100 &&
                impl_->imu_variance_.acce_var_norm < 100) {
                impl_->dr_params_.static_gyro_var = impl_->imu_variance_.gyro_var_norm;
                impl_->dr_params_.static_acc_var = impl_->imu_variance_.acce_var_norm;
            }
        }
    } else {
        impl_->reset_horizontal_flag_ = true;
        impl_->imu_avge_.InitAvg(kStaticTime);
    }
}

void DeadReckoning::QuaternionUpdate(const V3d &omega_i_b, const V3d &vel, double tau) {
    M3d cbn = impl_->quaternion_.toRotationMatrix();

    V3d omega_i_n, omega_b, omega_bT;

    double lat = impl_->latitude_;

    omega_i_n(0) = -vel(1) / RM(lat);
    omega_i_n(1) = kOMEGA_IE * COSL(lat) + vel(0) / RN(lat);
    omega_i_n(2) = kOMEGA_IE * SINL(lat) + vel(0) * TANL(lat) / RN(lat);

    omega_b = omega_i_b - (cbn.transpose()) * omega_i_n;

    omega_bT = omega_b * tau;

    Update(omega_bT);
}

void DeadReckoning::VelocityUpdate(V3d &vel, const V3d &fibb, double tau) {
    M3d cbn = impl_->quaternion_.toRotationMatrix();
    V3d fibn = cbn * fibb;
    V3d delABN = {0, 0, kGRAVITY};
    vel += (fibn - delABN) * tau;
}

void DeadReckoning::PositionUpdate(V3d &pos, const V3d &vel, double tau) {
    pos += 0.5 * (impl_->vel_psd_ + vel) * tau;
    impl_->vel_psd_ = vel;
}

void DeadReckoning::UpdateEndDrElements(const EndDrElements &end_dr_elements) {
    impl_->States_ = end_dr_elements.States;
    impl_->P_ = end_dr_elements.P;
    impl_->IKH_ = end_dr_elements.IKH;
    impl_->F_ = end_dr_elements.F;
    impl_->G_ = end_dr_elements.G;
    impl_->Q_ = end_dr_elements.Q;
    impl_->Qk_ = end_dr_elements.Qk;
    impl_->H_ = end_dr_elements.H;
    impl_->R_ = end_dr_elements.R;

    impl_->imu_avge_ = end_dr_elements.imu_avge;
    impl_->dr_data_ = end_dr_elements.dr_data;
    impl_->imu_data_ = end_dr_elements.imu_data;
    impl_->odom_data_ = end_dr_elements.odom_data;

    impl_->integral_odom[0] = end_dr_elements.integral_odom[0];
    impl_->integral_odom[1] = end_dr_elements.integral_odom[1];
    impl_->integral_odom[2] = end_dr_elements.integral_odom[2];
    impl_->clk_system_ = end_dr_elements.clk_system;
    impl_->imu_time_ms_psd_ = end_dr_elements.imu_time_ms_psd;

    impl_->imu_time_ros_psd_ = end_dr_elements.imu_time_ros_psd;
    impl_->odom_time_ros_psd_ = end_dr_elements.odom_time_ros_psd;
    impl_->velocity_odom_psd_ = end_dr_elements.velocity_odom_psd;
    impl_->delta_time_ms_ = end_dr_elements.delta_time_ms;
    impl_->vel_psd_ = end_dr_elements.vel_psd;

    impl_->imu_data_deque_ = end_dr_elements.imu_data_deque;
    impl_->dr_deque_ = end_dr_elements.dr_deque;
    impl_->imu_variance_ = end_dr_elements.imu_variance;

    impl_->quaternion_ = Att2Quat(impl_->dr_data_.attitude);
}

EndDrElements DeadReckoning::OutputEndDrElements() {
    EndDrElements end_dr_elements;
    end_dr_elements.States = impl_->States_;
    end_dr_elements.P = impl_->P_;
    end_dr_elements.IKH = impl_->IKH_;
    end_dr_elements.F = impl_->F_;
    end_dr_elements.G = impl_->G_;
    end_dr_elements.Q = impl_->Q_;
    end_dr_elements.Qk = impl_->Qk_;
    end_dr_elements.H = impl_->H_;
    end_dr_elements.R = impl_->R_;

    end_dr_elements.imu_avge = impl_->imu_avge_;
    end_dr_elements.dr_data = impl_->dr_data_;
    end_dr_elements.imu_data = impl_->imu_data_;
    end_dr_elements.odom_data = impl_->odom_data_;

    end_dr_elements.integral_odom[0] = impl_->integral_odom[0];
    end_dr_elements.integral_odom[1] = impl_->integral_odom[1];
    end_dr_elements.integral_odom[2] = impl_->integral_odom[2];
    end_dr_elements.clk_system = impl_->clk_system_;
    end_dr_elements.imu_time_ms_psd = impl_->imu_time_ms_psd_;

    end_dr_elements.imu_time_ros_psd = impl_->imu_time_ros_psd_;
    end_dr_elements.odom_time_ros_psd = impl_->odom_time_ros_psd_;
    end_dr_elements.velocity_odom_psd = impl_->velocity_odom_psd_;
    end_dr_elements.delta_time_ms = impl_->delta_time_ms_;
    end_dr_elements.vel_psd = impl_->vel_psd_;

    end_dr_elements.imu_data_deque = impl_->imu_data_deque_;
    end_dr_elements.dr_deque = impl_->dr_deque_;
    end_dr_elements.imu_variance = impl_->imu_variance_;

    return end_dr_elements;
}

void DeadReckoning::Update(V3d ang) {
    V4d qd, qt;

    double n = ang(0) * ang(0) + ang(1) * ang(1) + ang(2) * ang(2);
    double nn = n * n;
    double s = 0.5 - n / 48.0 + nn / 3840.0 - n * nn / 645120.0 + nn * nn / 185794560.0;
    qt(0) = 1.0 - n / 8.0 + nn / 384.0 - n * nn / 46080.0 + nn * nn / 10321920.0;
    qt(1) = s * ang(0);
    qt(2) = s * ang(1);
    qt(3) = s * ang(2);

    auto &q = impl_->quaternion_;
    qd(0) = q.w() * qt(0) - q.x() * qt(1) - q.y() * qt(2) - q.z() * qt(3);
    qd(1) = q.x() * qt(0) + q.w() * qt(1) - q.z() * qt(2) + q.y() * qt(3);
    qd(2) = q.y() * qt(0) + q.z() * qt(1) + q.w() * qt(2) - q.x() * qt(3);
    qd(3) = q.z() * qt(0) - q.y() * qt(1) + q.x() * qt(2) + q.w() * qt(3);

    double normQ = qd.norm();
    q.w() = qd(0) / normQ;
    q.x() = qd(1) / normQ;
    q.y() = qd(2) / normQ;
    q.z() = qd(3) / normQ;
}

DrElements DeadReckoning::GetDRResult() {
    if (impl_->dr_deque_.empty()) {
        DrElements null_ele;
        null_ele.valid = false;
        return null_ele;
    } else {
        return impl_->dr_deque_.back();
    }
}

}  // namespace mapping::core
