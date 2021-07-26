//
// Created by herenjie on 2021/2/1.
//

#ifndef SCRUBBER_SLAM_DR_PRE_INTEGRATION_H
#define SCRUBBER_SLAM_DR_PRE_INTEGRATION_H

#include <ros/ros.h>
#include <deque>
#include "common/message_def.h"
#include "dr_constant.h"
#include "common/global_config.h"

namespace scrubber_slam{
    struct ImuState{
        double imu_time = -1;
        Eigen::Quaterniond quaterniond_angle = Eigen::Quaterniond(1.0,0.,0.,0.);    ///四元数角度
        Vec3 velocity = Vec3::Zero();                                                          ///速度
        Vec3 position = Vec3::Zero();                                                          ///位置坐标
        Vec3 bg = Vec3::Zero();                                                                ///imu角速度零偏
        Vec3 ba = Vec3::Zero();                                                                ///imu加速度零偏
    };
    struct PreIntegrationState{
        double start_imu_time = -1;
        double end_imu_time = -1;
        Eigen::Quaterniond  delta_q{1.,0.,0.,0.};
        Vec3                delta_velocity = Vec3::Zero();
        Vec3                delta_position = Vec3::Zero();
    };

    class DRPreIntegration {
    public:
        DRPreIntegration();
        ~DRPreIntegration();
        void AddImu(const std::shared_ptr<UImuMsg>& imu);
        void AddFrontWheelSpeed(const FrontWheelSpeedMsgScru::ConstPtr &wheelspeed);
        void AddRearWheelSpeed(UWheelSpeedMsgPtr wheelspeed);
        void GetPreIntegration(Eigen::Quaterniond &delta_q, Eigen::Vector3d& delta_p);
        bool DrInited(){return dr_inited_;}
        bool GetLastPose(SE3& pose);
        ImuState GetLastState(){
            return last_imu_state_;
        };
        void Reset(Eigen::Quaterniond quaterniond_angle, Vec3 position){
            std::unique_lock<std::mutex> ism(imu_state_mutex_);
            last_imu_state_.quaterniond_angle = quaterniond_angle;    ///四元数角度
            last_imu_state_.velocity = Vec3::Zero();                                                          ///速度
            last_imu_state_.position = position;                                                          ///位置坐标
        }
        void GetWheelspeed(Vec3& wheelspeed_speed){
            std::unique_lock<std::mutex> ism(wheelspeed_speed_mutex_);
            wheelspeed_speed = wheelspeed_speed_;
        };
    private:
        void PubPose(const SE3& pose);
        ros::Publisher pub_pose_;

        ///初始化相关
        void InitImuBias();
        void InitRollPitchByImu();
        bool        dr_inited_ = false;
        std::mutex  imu_vec_init_mutex_;
        std::vector<std::shared_ptr<UImuMsg>>
                    imu_vec_init_;
        Vec3        bg_ = Vec3::Zero();                                                                ///imu角速度零偏
        Vec3        ba_ = Vec3::Zero();                                                                ///imu加速度零偏
        Vec3        acc_avr_init_ = Vec3::Zero();                                                      ///imu加速度初始静止均值，用于求解初始俯仰横滚
        std::mutex  imu_state_mutex_;
        ImuState    last_imu_state_;
        ///long term dr递推状态
        std::mutex  wheelspeed_speed_mutex_;
        Vec3        wheelspeed_speed_ = Vec3::Zero();
        ///预积分参数
/***
* Pwj = Pwi + Qwi * 积分(q * v * dt)
* Qwj = Qwi * 积分(q * [1,0.5w] *dt)
* 状态量为：　j时刻[Pwj， Qwj] i时刻[Pwi, Qwi]
* 本函数将两时刻之间的imu和轮速计测量转换为预积分形式，用于dr和激光匹配的紧耦合优化
* delta_p　＝　积分(q * v * dt)
* delta_q　＝　积分(q * [0, 0.5w] *dt)
* */
        std::mutex          preintegration_mutex_;
        Eigen::Quaterniond  delta_q_{1.,0.,0.,0.};
        Vec3                delta_position_ = Vec3::Zero();//alpha
        Vec3                delta_velocity_ = Vec3::Zero();//beta
        Vec3                last_angular_velocity_ = Vec3::Zero();                                             ///上一时刻IMU角速度，用于中值积分
        Vec3                last_linear_acceleration_ = Vec3::Zero();                                          ///上一时刻IMU加速度，用于中值积分
        std::mutex          id_deque_mutex_;
        std::deque<std::shared_ptr<UImuMsg>>
                            imu_deque_;
        std::mutex          ws_deque_mutex_;
        std::deque<FrontWheelSpeedMsgScru::ConstPtr>
                            wheelspeed_deque_;
        float wheel_radius_ = 0.15;
    };
}



#endif //SCRUBBER_SLAM_DR_PRE_INTEGRATION_H
