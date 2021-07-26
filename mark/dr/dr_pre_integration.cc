//
// Created by herenjie on 2021/2/1.
//

#include "dr/dr_pre_integration.h"

namespace scrubber_slam{
    DRPreIntegration::DRPreIntegration(){
        ros::NodeHandle nh;
        pub_pose_ = nh.advertise<geometry_msgs::PoseStamped> ("/tracking/my_dr_pose", 1);
        auto yaml = GlobalConfig::Get();
        wheel_radius_ = yaml->GetValue<float>("dr", "wheel_radius");
    }
    DRPreIntegration::~DRPreIntegration(){}

    void DRPreIntegration::AddImu(const std::shared_ptr<UImuMsg>& imu){
        imu_deque_.emplace_back(imu);

        double timestamp = imu->timestamp;
        double time_ms = imu->time_tag;
        Vec3 angular_velocity = Vec3::Zero();///此刻IMU角速度
        Vec3 linear_acceleration = Vec3::Zero();///此刻IMU加速度
        angular_velocity(0) = imu->gyro_x * kDEG2RAD;
        angular_velocity(1) = imu->gyro_y * kDEG2RAD;
        angular_velocity(2) = imu->gyro_z * kDEG2RAD;
        linear_acceleration(0) = imu->acce_x;
        linear_acceleration(1) = imu->acce_y;  // unit m/s^2
        linear_acceleration(2) = imu->acce_z;

        if(!dr_inited_){
            std::unique_lock<std::mutex> ivim(imu_vec_init_mutex_);
            imu_vec_init_.emplace_back(imu);
            if(imu_vec_init_.size()>200){
                InitImuBias();
                InitRollPitchByImu();
                dr_inited_ = true;
            } else{
                last_angular_velocity_ = angular_velocity;         ///上一时刻IMU角速度，用于中值积分
                last_linear_acceleration_ = linear_acceleration;   ///上一时刻IMU加速度，用于中值积分
                return;
            }
        }

        double dt = 0.01;
        {
            std::unique_lock<std::mutex> ism(imu_state_mutex_);
            last_imu_state_.imu_time = imu->timestamp;

            ///1.角度更新
            Vec3 mid_angular_velocity = 0.5 * ( last_angular_velocity_ + angular_velocity );///此刻IMU角速度
            last_angular_velocity_ = angular_velocity;
            last_imu_state_.quaterniond_angle = last_imu_state_.quaterniond_angle * Eigen::Quaterniond(1,
                                                                                                       0.5*(mid_angular_velocity[0] - bg_[0]) * dt ,
                                                                                                       0.5*(mid_angular_velocity[1] - bg_[1]) * dt,
                                                                                                       0.5*(mid_angular_velocity[2] - bg_[2]) * dt);
            ///2.位置更新
            last_imu_state_.position = last_imu_state_.position + last_imu_state_.quaterniond_angle * wheelspeed_speed_ * dt;

            ///预积分变量
            {
                std::unique_lock<std::mutex> pm(preintegration_mutex_);
                delta_position_ += delta_q_ * wheelspeed_speed_ * dt;
                delta_q_ = delta_q_ * Eigen::Quaterniond(1,
                                               0.5*(mid_angular_velocity[0] - bg_[0]) * dt ,
                                               0.5*(mid_angular_velocity[1] - bg_[1]) * dt,
                                               0.5*(mid_angular_velocity[2] - bg_[2]) * dt);
                delta_q_.normalize();
            }
        }

        SE3 cur_dr_pose = SE3(last_imu_state_.quaterniond_angle, last_imu_state_.position);
        PubPose(cur_dr_pose);
    }

    void DRPreIntegration::AddFrontWheelSpeed(const FrontWheelSpeedMsgScru::ConstPtr &wheelspeed){
//        LOG(INFO)<<"前轮速计";
        wheelspeed_deque_.emplace_back(wheelspeed);
        if(!dr_inited_){
            std::unique_lock<std::mutex> ivim(imu_vec_init_mutex_);
            if(wheelspeed->wheelspeed_f_pulse >3){
                imu_vec_init_.clear();
            }
        }

        double wheel_dist = wheelspeed->wheelspeed_f_pulse / 2650.0 * 2 * M_PI * 0.125 ;///前轮轮前进距离，因为topic重复发了两次，所以 * 0.5
        double front_angle = (double)wheelspeed->real_str_angle;                             ///前轮与车身角度
        double dist_x = wheel_dist * std::cos((front_angle-10)/308.407643312);                 ///车体前进的距离
//        double dist_x = wheel_dist * std::cos((front_angle)/312.738853503);                 ///车体前进的距离

        Vec3 wheel_speed(dist_x * 10,0,0);

        {
            std::unique_lock<std::mutex> ism(imu_state_mutex_);
            std::unique_lock<std::mutex> wsm(wheelspeed_speed_mutex_);

            wheelspeed_speed_ = wheel_speed;
//            last_imu_state_.position = wheelspeed_position_;///用车轮位置替换imu位置状态，todo:ekf融合
//            last_imu_state_.velocity = last_imu_state_.quaterniond_angle * wheel_speed;///用车轮位置替换imu位置状态，todo:ekf融合
        }
    }

    void DRPreIntegration::AddRearWheelSpeed(UWheelSpeedMsgPtr wheelspeed){
//        LOG(INFO)<<"后轮速计";
        if(!dr_inited_){
            std::unique_lock<std::mutex> ivim(imu_vec_init_mutex_);
            if(wheelspeed->wheelspeed_lr_pluse >3 || wheelspeed->wheelspeed_rr_pluse >3){
                imu_vec_init_.clear();
            }
        }

//        double velocity_l = WHEEL_RADIUS * wheelspeed->wheelspeed_lr_pluse / CIRCLE_PULSE * 2 * M_PI / tODOM_SPAN;
//        double velocity_r = WHEEL_RADIUS * wheelspeed->wheelspeed_rr_pluse / CIRCLE_PULSE * 2 * M_PI / tODOM_SPAN;
      double velocity_l = wheel_radius_ * wheelspeed->wheelspeed_lr_pluse / CIRCLE_PULSE * 2 * M_PI / tODOM_SPAN;
      double velocity_r = wheel_radius_ * wheelspeed->wheelspeed_rr_pluse / CIRCLE_PULSE * 2 * M_PI / tODOM_SPAN;
        double vel_x = 0.5 * (velocity_l + velocity_r);

        Vec3 wheel_speed(vel_x,0,0);
        {
            std::unique_lock<std::mutex> ism(imu_state_mutex_);
            std::unique_lock<std::mutex> wsm(wheelspeed_speed_mutex_);

            wheelspeed_speed_ = wheel_speed;
//            last_imu_state_.position = wheelspeed_position_;///用车轮位置替换imu位置状态，todo:ekf融合
//            last_imu_state_.velocity = last_imu_state_.quaterniond_angle * wheel_speed;///用车轮位置替换imu位置状态，todo:ekf融合
        }
    }

    void DRPreIntegration::InitImuBias(){
        for(auto &msg: imu_vec_init_){
            Vec3 angular_velocity = Vec3::Zero();///此刻IMU角速度
            Vec3 linear_acceleration = Vec3::Zero();///此刻IMU加速度
            angular_velocity(0) = msg->gyro_x * kDEG2RAD;
            angular_velocity(1) = msg->gyro_y * kDEG2RAD;
            angular_velocity(2) = msg->gyro_z * kDEG2RAD;
            linear_acceleration(0) = msg->acce_x;
            linear_acceleration(1) = msg->acce_y;  // unit m/s^2
            linear_acceleration(2) = msg->acce_z;

            bg_ += angular_velocity;
            acc_avr_init_ += linear_acceleration;
        }
        bg_ /= double(imu_vec_init_.size());
        acc_avr_init_ /= double(imu_vec_init_.size());
    }

    void DRPreIntegration::InitRollPitchByImu(){
        std::unique_lock<std::mutex> ism(imu_state_mutex_);
        Eigen::Vector3d euler_angle_init(0.0,0.,0.0);
        euler_angle_init(0) = atan2(-acc_avr_init_(0), acc_avr_init_(2));
        euler_angle_init(1) = asin(acc_avr_init_(1) / kGRAVITY);

        last_imu_state_.quaterniond_angle = Eigen::AngleAxisd(euler_angle_init[0], ::Eigen::Vector3d::UnitZ()) *
                                            Eigen::AngleAxisd(euler_angle_init[1], ::Eigen::Vector3d::UnitY()) *
                                            Eigen::AngleAxisd(euler_angle_init[2], ::Eigen::Vector3d::UnitX());
        last_imu_state_.ba = ba_;
        last_imu_state_.bg = bg_;
        LOG(INFO)<<acc_avr_init_;
        LOG(INFO)<<"g: "<<acc_avr_init_.norm();
        LOG(INFO)<<"euler_angle_init: "<<euler_angle_init.transpose();
    }

    bool DRPreIntegration::GetLastPose(SE3& pose){
        if(!dr_inited_){
            LOG(WARNING)<<"MLDR not init yet...";
            return false;
        }

        std::unique_lock<std::mutex> ism(imu_state_mutex_);
        pose = SE3(last_imu_state_.quaterniond_angle, last_imu_state_.position);

        return true;
    }

    void DRPreIntegration::GetPreIntegration(Eigen::Quaterniond &delta_q, Eigen::Vector3d& delta_p){
        std::unique_lock<std::mutex> pm(preintegration_mutex_);
        delta_q = delta_q_;
        delta_p = delta_position_;
        delta_position_ = Vec3::Zero();
        delta_q_ = Eigen::Quaterniond(1.,0.,0.,0.);
    }

    void DRPreIntegration::PubPose(const SE3& pose){
        geometry_msgs::PoseStamped cur_pose_ros_dr;
        cur_pose_ros_dr.header.seq = 0;
        cur_pose_ros_dr.header.stamp =ros::Time::now();//如果有问题就使用Time(0)获取时间戳，确保类型一致
        cur_pose_ros_dr.header.frame_id = "map";
        cur_pose_ros_dr.pose.position.x = pose.translation()[0];
        cur_pose_ros_dr.pose.position.y = pose.translation()[1];
        cur_pose_ros_dr.pose.position.z = pose.translation()[2];
        cur_pose_ros_dr.pose.orientation.x = pose.unit_quaternion().x();
        cur_pose_ros_dr.pose.orientation.y = pose.unit_quaternion().y();
        cur_pose_ros_dr.pose.orientation.z = pose.unit_quaternion().z();
        cur_pose_ros_dr.pose.orientation.w = pose.unit_quaternion().w();
        pub_pose_.publish(cur_pose_ros_dr);
    }
}