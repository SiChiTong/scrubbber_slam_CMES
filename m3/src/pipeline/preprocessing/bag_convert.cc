//
// Created by gaoxiang on 2020/8/11.
//

#include "pipeline/preprocessing/bag_convert.h"
#include "common/common_func.h"
#include "common/message_def.h"
#include "common/topics_def.h"
#include "io/file_io.h"

#include <glog/logging.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace mapping::pipeline {

#define SPEED2PULSE 105.144943049;

// 通用消息拷贝函数
template <typename T>
void GeneralGPSMsgConvert(const T &old_msg, GpsMsg &new_msg) {
    new_msg.header = old_msg.header;
    new_msg.lon = old_msg.lon;
    new_msg.lat = old_msg.lat;
    new_msg.height = old_msg.height;
    new_msg.heading = old_msg.heading;
    new_msg.velocity = old_msg.velocity;
    new_msg.track_angle = old_msg.track_angle;
    new_msg.utctime = old_msg.utctime;
    new_msg.hdop = old_msg.hdop;
    new_msg.diff_age = old_msg.diff_age;
    new_msg.base_length = old_msg.base_length;
    new_msg.heading_std = old_msg.heading_std;
    new_msg.status = old_msg.status;
    new_msg.satenum = old_msg.satenum;
    new_msg.is_heading_valid = old_msg.is_heading_valid;
}

/// LADS 特化，使用UTC时间
template <>
void GeneralGPSMsgConvert(const LadsGpsMsg &old_msg, GpsMsg &new_msg) {
    double utc_expand = (double)old_msg.utctime / 2.5e-4;
    double unix_time = common::Utc2Unix((int)old_msg.week, utc_expand);
    new_msg.header = old_msg.header;
    new_msg.header.stamp = ros::Time(unix_time);
    new_msg.lon = old_msg.lon;
    new_msg.lat = old_msg.lat;
    new_msg.height = old_msg.height;
    new_msg.heading = old_msg.heading;
    new_msg.velocity = old_msg.velocity;
    new_msg.track_angle = old_msg.track_angle;
    new_msg.utctime = old_msg.utctime;
    new_msg.hdop = old_msg.hdop;
    new_msg.diff_age = old_msg.diff_age;
    new_msg.base_length = old_msg.base_length;
    new_msg.heading_std = old_msg.heading_std;
    new_msg.status = old_msg.status;
    new_msg.satenum = old_msg.satenum;
    new_msg.is_heading_valid = old_msg.is_heading_valid;
}

template <typename T>
void GeneralIMUMsgConvert(const T &old_msg, ImuMsg &new_msg) {
    new_msg.header = old_msg.header;
    new_msg.TimeTag = old_msg.TimeTag;
    new_msg.utcTime = old_msg.utcTime;
    new_msg.gyro_x = old_msg.gyro_x;
    new_msg.gyro_y = old_msg.gyro_y;
    new_msg.gyro_z = old_msg.gyro_z;
    new_msg.acce_x = old_msg.acce_x;
    new_msg.acce_y = old_msg.acce_y;
    new_msg.acce_z = old_msg.acce_z;
    new_msg.Temperature = old_msg.Temperature;
}

template <>
void GeneralIMUMsgConvert(const LadsImuMsg &old_msg, ImuMsg &new_msg) {
    new_msg.header = old_msg.header;

    double utc_expand = (double)old_msg.utcTime / 2.5e-4;
    double unix_time = common::Utc2Unix((int)old_msg.week, utc_expand);
    new_msg.header.stamp = ros::Time(unix_time);

    new_msg.TimeTag = old_msg.TimeTag;
    new_msg.utcTime = old_msg.utcTime;
    new_msg.gyro_x = old_msg.gyro_x;
    new_msg.gyro_y = old_msg.gyro_y;
    new_msg.gyro_z = old_msg.gyro_z;
    new_msg.acce_x = old_msg.acce_x;
    new_msg.acce_y = old_msg.acce_y;
    new_msg.acce_z = old_msg.acce_z;
    new_msg.Temperature = old_msg.Temperature;
}

template <typename T>
void GeneralOdomMsgConvert(const T &old_msg, OdomMsg &new_msg) {
    new_msg.header = old_msg.header;
    new_msg.wheelspeed_lr_pluse = old_msg.wheelspeed_lr_pluse;
    new_msg.wheelspeed_rr_pluse = old_msg.wheelspeed_rr_pluse;
    new_msg.pluse_mask = old_msg.pluse_mask;
}

// LADS特化
template <>
void GeneralOdomMsgConvert(const LadsOdomMsg &old_msg, OdomMsg &new_msg) {
    new_msg.header = old_msg.header;
    new_msg.wheelspeed_lr_pluse = old_msg.wheelspeed_lr_pluse;
    new_msg.wheelspeed_rr_pluse = old_msg.wheelspeed_rr_pluse;
    new_msg.pluse_mask = old_msg.pluse_mask;

    double utc_expand = (double)old_msg.utcTime / 2.5e-4;
    double unix_time = common::Utc2Unix((int)old_msg.week, utc_expand);

    new_msg.header = old_msg.header;
    new_msg.header.stamp = ros::Time(unix_time);

    new_msg.wheelspeed_lr_pluse = (int)((old_msg.wheelspeed_lr_pluse * 0.1 * 1024) / (2 * M_PI * 0.155));
    new_msg.wheelspeed_rr_pluse = (int)((old_msg.wheelspeed_rr_pluse * 0.1 * 1024) / (2 * M_PI * 0.155));
    new_msg.pluse_mask = old_msg.pluse_mask;
}

template <>
void GeneralOdomMsgConvert(const CanMsg &old_msg, OdomMsg &new_msg) {
    new_msg.header = old_msg.header;
    new_msg.wheelspeed_lr_pluse = old_msg.wheel_speed.wheelspeed_lr * SPEED2PULSE;
    new_msg.wheelspeed_rr_pluse = old_msg.wheel_speed.wheelspeed_rr * SPEED2PULSE;
    new_msg.pluse_mask = old_msg.wheel_speed.pluse_mask;
}

template <typename T>
void GeneralPacketsMsgConvert(const T &old_msg, PacketsMsg &new_msg) {
    new_msg.header = old_msg.header;
    new_msg.header.frame_id = "velodyne_0";
    new_msg.fault_vec.info_vec = {};
    new_msg.fault_vec.module_fault_level = 0;
    new_msg.packets = old_msg.packets;
}

// LADS特化
template <>
void GeneralPacketsMsgConvert(const LadsPacketsMsg &old_msg, PacketsMsg &new_msg) {
    new_msg.header = old_msg.header;
    new_msg.header.frame_id = "velodyne_0";
    new_msg.fault_vec.info_vec = {};
    new_msg.fault_vec.module_fault_level = 0;
    new_msg.packets = old_msg.packets;

    new_msg.header.stamp = ros::Time(old_msg.header.stamp.toSec() + 0.050);
}

template <typename T>
void GeneralSuTengIRADPacketsMsgConvert(const T &old_msg, SuTengIRADPacketsMsg &new_msg) {
    new_msg = old_msg;
}

// LADS特化
template <>
void GeneralSuTengIRADPacketsMsgConvert(const SuTengIRADPacketsMsg &old_msg, SuTengIRADPacketsMsg &new_msg) {
    new_msg = old_msg;
    new_msg.header.stamp = ros::Time(old_msg.header.stamp.toSec() + 0.050);
    return;
}

BagConvert::BagConvert() {}

bool BagConvert::Run(const std::string &in_file, const std::string &out_file) {
    in_path_ = in_file;
    out_path_ = out_file;

    if (io::PathExists(in_file) == false) {
        LOG(ERROR) << "cannot find file: " << in_file;
        return false;
    }

    // reindex
    ReIndexBag();

    // run convert
    DoBagConvert();

    return true;
}

void BagConvert::ReIndexBag() {
    std::string cmd = "rosbag reindex " + in_path_;
    LOG(INFO) << "cmd: " << cmd;
    system(cmd.c_str());

    // remove the origin bag
    cmd = "rm " + in_path_.substr(0, in_path_.size() - 3) + std::string("orig.bag");
    LOG(INFO) << "cmd: " << cmd;
    system(cmd.c_str());

    return;
}

void BagConvert::DoBagConvert() {
    /// 以下定义各种消息的转换函数，大部分消息使用直连或通用转换，部分app功能相关的需要重新定义
    /// NOTE 这段必须放在DoBagConvert内部，否则会过期
    process_funcs_.clear();
    process_funcs_ = {
        MakeDirectPassFunc<GpsMsg>(common::topics::tp_gps),          // 原始GPS无须转换
        MakeDirectPassFunc<ImuMsg>(common::topics::tp_imu),          // 原始IMU无须转换
        MakeDirectPassFunc<OdomMsg>(common::topics::tp_wheelspeed),  // 原Odom无须转换
        MakeDirectPassFunc<PacketsMsg>(common::topics::tp_lidar),    // 原Lidar无须转换
        MakeDirectPassFunc<AppWxbMsg>(common::topics::tp_appwxb),    // 原wxb app无须转换
        MakeDirectPassFunc<AppWbdMsg>(common::topics::tp_appwbd),    // 原wbd app无须转换
        MakeDirectPassFunc<ImageMsg>(common::topics::tp_image),      // 原image app无须转换

        // new 系列
        MakeConvertFunc<NewPacketsMsg, PacketsMsg>(common::topics::tp_lidar,
                                                   GeneralPacketsMsgConvert<NewPacketsMsg>),  // New packets转换
        MakeConvertFunc<NewOdomMsg, OdomMsg>(common::topics::tp_wheelspeed,
                                             GeneralOdomMsgConvert<NewOdomMsg>),  // New odom转换
        MakeConvertFunc<NewGpsMsg, GpsMsg>(common::topics::tp_gps,
                                           GeneralGPSMsgConvert<NewGpsMsg>),  // New GPS

        // avos_x 系列
        MakeConvertFunc<AxGpsMsg, GpsMsg>(common::topics::tp_gps, GeneralGPSMsgConvert<AxGpsMsg>),
        MakeConvertFunc<AxImuMsg, ImuMsg>(common::topics::tp_imu, GeneralIMUMsgConvert<AxImuMsg>),
        MakeConvertFunc<AxNewGpsMsg, GpsMsg>(common::topics::tp_gps, GeneralGPSMsgConvert<AxNewGpsMsg>),
        MakeConvertFunc<AxNewImuMsg, ImuMsg>(common::topics::tp_imu, GeneralIMUMsgConvert<AxNewImuMsg>),
        MakeConvertFunc<NewAppWxbMsg>(common::topics::tp_appwxb,
                                      [](const NewAppWxbMsg &old_msg, AppWxbMsg &new_msg) {
                                          new_msg.specialareaswitch = old_msg.specialareaswitch;
                                          new_msg.specialareatype = old_msg.specialareatype;
                                          new_msg.specialareaissave = 255;
                                      }),
        MakeConvertFunc<NewAppWbdMsg>(common::topics::tp_appwbd,
                                      [](const NewAppWbdMsg &old_msg, AppWbdMsg &new_msg) {
                                          new_msg.open_box_num = old_msg.open_box_num;
                                          new_msg.request_code = old_msg.request_code;
                                          new_msg.map_collect_flag = old_msg.map_collect_flag;
                                          new_msg.map_name = old_msg.map_name;
                                          new_msg.special_point_type = old_msg.special_point_type;
                                          new_msg.u_op = old_msg.u_op;
                                          new_msg.manaul_control = old_msg.manaul_control;
                                          new_msg.calibration = 0;
                                          new_msg.wake_up = 0;
                                      }),
        MakeConvertFunc<XAppWxbMsg>(common::topics::tp_appwxb,
                                    [](const XAppWxbMsg &old_msg, AppWxbMsg &new_msg) {
                                        std::string str = old_msg.specialareatype;
                                        new_msg.specialareaswitch = old_msg.specialareaswitch;
                                        if (" " != str) {
                                            std::string re = (str.substr(0, 1)).c_str();
                                            char a;
                                            if (re.size() < 1) {
                                                return;
                                            } else {
                                                a = re.at(0);
                                            }
                                            std::string number_str = str.substr(1, str.size() - 1);
                                            int number = atoi(number_str.c_str());
                                            int type = 0;
                                            if (old_msg.specialareaissave == 255) {
                                                switch (a) {
                                                    case 'a': {
                                                        type = 0;
                                                        break;
                                                    }
                                                    case 'b': {
                                                        type = 1;
                                                        break;
                                                    }
                                                    case 'c': {
                                                        type = 2;
                                                        break;
                                                    }
                                                    case 'd': {
                                                        type = 3;
                                                        break;
                                                    }
                                                    default: {
                                                        type = 255;
                                                        break;
                                                    }
                                                }
                                                new_msg.specialareatype = type;
                                                new_msg.specialareaissave = 255;
                                            } else {
                                                switch (a) {
                                                    case 'a': {
                                                        if (0 == old_msg.specialareaissave) {
                                                            type = 4;
                                                        } else {
                                                            type = 5;
                                                        }
                                                        break;
                                                    }
                                                    case 'b': {
                                                        if (0 == old_msg.specialareaissave) {
                                                            type = 6;
                                                        } else {
                                                            type = 7;
                                                        }
                                                        break;
                                                    }
                                                    case 'c': {
                                                        if (0 == old_msg.specialareaissave) {
                                                            type = 8;
                                                        } else {
                                                            type = 9;
                                                        }
                                                        break;
                                                    }
                                                    case 'd': {
                                                        if (0 == old_msg.specialareaissave) {
                                                            type = 10;
                                                        } else {
                                                            type = 11;
                                                        }
                                                        break;
                                                    }
                                                    default: {
                                                        type = 255;
                                                        number = 255;
                                                        break;
                                                    }
                                                }
                                                new_msg.specialareatype = type;
                                                new_msg.specialareaissave = number;
                                            }
                                        }
                                    }),
        MakeDirectPassFunc<LidarPoseMsg>(common::topics::tp_lidarpose),  // 原lidar_pose无须转换
        MakeDirectPassFunc<LocalPoseMsg>(common::topics::tp_localpose),  // 原local_pose无须转换

        // LADS 相关
        MakeConvertFunc<LadsGpsMsg, GpsMsg>(common::topics::tp_gps,
                                            GeneralGPSMsgConvert<LadsGpsMsg>),  // LADS gps转换
        MakeConvertFunc<LadsImuMsg, ImuMsg>(common::topics::tp_imu,
                                            GeneralIMUMsgConvert<LadsImuMsg>),  // LADS imu转换
        MakeConvertFunc<LadsOdomMsg, OdomMsg>(common::topics::tp_wheelspeed, GeneralOdomMsgConvert<LadsOdomMsg>),
        MakeConvertFunc<CanMsg, OdomMsg>(common::topics::tp_cansensor, GeneralOdomMsgConvert<CanMsg>),
        MakeConvertFunc<LadsPacketsMsg, PacketsMsg>(common::topics::tp_lidar_velodyne_lads,
                                                    GeneralPacketsMsgConvert<LadsPacketsMsg>),  // LADS packets转换

        // 其他雷达
        MakeDirectPassFunc<HeSaiScanMsg>(common::topics::tp_lidar_hesai),               // 禾赛
        MakeDirectPassFunc<SuTengScanMsg>(common::topics::tp_lidar_suteng_scan),        // 速腾scan
        MakeDirectPassFunc<SuTengPacketsMsg>(common::topics::tp_lidar_suteng_packets),  // 官方速腾packets
        // MakeDirectPassFunc<SuTengIRADPacketsMsg>(common::topics::tp_lidar_suteng_scan),  // 自研速腾packets
        MakeConvertFunc<SuTengIRADPacketsMsg, SuTengIRADPacketsMsg>(common::topics::tp_lidar_suteng_scan, GeneralSuTengIRADPacketsMsgConvert<SuTengIRADPacketsMsg>),  // 自研速腾packets
    };

    rosbag::Bag bag;
    bag.open(in_path_, rosbag::bagmode::Read);

    rosbag::Bag bag_out;
    bag_out.open(out_path_, rosbag::bagmode::Write);

    std::vector<int> process_msgs(process_funcs_.size());
    for (const rosbag::MessageInstance &m : rosbag::View(bag)) {
        for (auto &process_func : process_funcs_) {
            if (process_func(m, bag_out)) {
                break;
            }
        }
    }

    bag.close();
    bag_out.close();
    LOG(INFO) << "convert done.";
}

}  // namespace mapping::pipeline
