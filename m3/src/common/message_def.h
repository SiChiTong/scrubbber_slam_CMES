//
// Created by gaoxiang on 2020/8/10.
//

#ifndef MAPPING_MESSAGE_DEF_H
#define MAPPING_MESSAGE_DEF_H

// ROS
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>

// self-defined messages
#include "../../../build/devel/include/velodyne_msgs/VelodyneScan.h"
#include "common_msgs/fault_vec.h"
#include "ivactuator/ivwheelspeed.h"
#include "ivapp/app2taskcentral.h"
#include "ivapp/specialfunctionpoint.h"
#include "ivcanbus/ivcansensor.h"
#include "ivcanbus/ivwheelspeed.h"
#include "ivlocmsg/ivmsglocpos.h"
#include "ivlocmsg/ivmsglocpos_with_fault.h"
#include "ivlocmsg/ivsensorgps.h"
#include "ivlocmsg/ivsensorimu.h"
#include "ivlocmsg/ndt_status.h"
#include "monitor_msgs/fault_vec.h"

#include "avos_new/ivsensorgps.h"
#include "avos_new/ivsensorimu.h"
#include "avos_x/ivsensorgps.h"
#include "avos_x/ivsensorimu.h"

#include "new_ivapp/app2taskcentral.h"
#include "new_ivapp/specialfunctionpoint.h"
#include "new_ivsensorgps/ivsensorgps.h"
#include "new_velodyne_msgs/VelodyneScan.h"
#include "x_ivapp/specialfunctionpoint.h"

#include "mainstream_msgs/ivmsglocalization.h"

#include "lads/ivsensorgps.h"
#include "lads/ivsensorimu.h"
#include "lads/ivwheelspeed.h"
#include "lads_velodyne_msgs/VelodyneScanUnified.h"

#include "common/mapping_point_types.h"

// 禾赛激光
#include "thirdparty/hesai/include/pandarGeneral_sdk.h"

// 速腾激光
#include "thirdparty/suteng/include/manager.h"
#include "thirdparty/suteng_IRAD/include/rs_lidar_api.h"

// 消息定义
/// msg for mapping
using GpsMsg = ivlocmsg::ivsensorgps;
using ImuMsg = ivlocmsg::ivsensorimu;
using OdomMsg = ivactuator::ivwheelspeed;
using CanMsg = ivcanbus::ivcansensor;
using AppWxbMsg = ivapp::specialfunctionpoint;
using AppWbdMsg = ivapp::app2taskcentral;
using TimedAppWxbMsg = std::pair<std_msgs::Header, AppWxbMsg>;

/// TODO PacketsMsg不要和velodyne绑定
using PacketsMsg = velodyne_msgs::VelodyneScan;
using ImageMsg = sensor_msgs::CompressedImage;
using PointsMsg = pcl::PointCloud<mapping::common::PointXYZIT>;

using NewPacketsMsg = new_velodyne_msgs::VelodyneScan;
using NewGpsMsg = new_ivsensorgps::ivsensorgps;
using NewOdomMsg = ivcanbus::ivwheelspeed;

/// avos_x
using AxGpsMsg = avos_x::ivsensorgps;
using AxImuMsg = avos_x::ivsensorimu;
using AxNewGpsMsg = avos_new::ivsensorgps;
using AxNewImuMsg = avos_new::ivsensorimu;
using NewAppWxbMsg = new_ivapp::specialfunctionpoint;
using NewAppWbdMsg = new_ivapp::app2taskcentral;
using XAppWxbMsg = x_ivapp::specialfunctionpoint;
using LidarPoseMsg = ivlocmsg::ndt_status;
using LocalPoseMsg = mainstream_msgs::ivmsglocalization;

// lads
using LadsGpsMsg = lads::ivsensorgps;
using LadsImuMsg = lads::ivsensorimu;
using LadsOdomMsg = lads::ivwheelspeed;
using LadsPacketsMsg = lads_velodyne_msgs::VelodyneScanUnified;

/// Ptrs
using GpsMsgPtr = boost::shared_ptr<GpsMsg>;
using ImuMsgPtr = boost::shared_ptr<ImuMsg>;
using OdomMsgPtr = boost::shared_ptr<OdomMsg>;
using CanMsgPtr = boost::shared_ptr<CanMsg>;
using AppWxbMsgPtr = boost::shared_ptr<AppWxbMsg>;
using AppWbdMsgPtr = boost::shared_ptr<AppWbdMsg>;
using TimedAppWxbMsgPtr = boost::shared_ptr<TimedAppWxbMsg>;
using PacketsMsgPtr = boost::shared_ptr<PacketsMsg>;
using ImageMsgPtr = boost::shared_ptr<ImageMsg>;
using PointsMsgPtr = boost::shared_ptr<PointsMsg>;

using NewPacketsMsgPtr = boost::shared_ptr<NewPacketsMsg>;
using NewGpsMsgPtr = boost::shared_ptr<NewGpsMsg>;
using NewOdomMsgPtr = boost::shared_ptr<NewOdomMsg>;

/// avos_x
using AxGpsMsgPtr = boost::shared_ptr<AxGpsMsg>;
using AxImuMsgPtr = boost::shared_ptr<AxImuMsg>;
using AxNewGpsMsgPtr = boost::shared_ptr<AxNewGpsMsg>;
using AxNewImuMsgPtr = boost::shared_ptr<AxNewImuMsg>;
using NewAppWxbMsgPtr = boost::shared_ptr<NewAppWxbMsg>;
using NewAppWbdMsgPtr = boost::shared_ptr<NewAppWbdMsg>;
using XAppWxbMsgPtr = boost::shared_ptr<XAppWxbMsg>;

using LidarPoseMsgPtr = boost::shared_ptr<LidarPoseMsg>;
using LocalPoseMsgPtr = boost::shared_ptr<LocalPoseMsg>;

// lads
using LadsGpsMsgPtr = boost::shared_ptr<LadsGpsMsg>;
using LadsImuMsgPtr = boost::shared_ptr<LadsImuMsg>;
using LadsOdomMsgPtr = boost::shared_ptr<LadsOdomMsg>;
using LadsPacketsMsgPtr = boost::shared_ptr<LadsPacketsMsg>;

// ===================================================================
// 各家激光雷达
// 速腾
using SuTengScanMsg = rslidar_msgs::rslidarScan;
using SuTengPacketsMsg = rslidar_msgs::rslidarPacket;
using SuTengIRADPacketsMsg = suteng_irad::LidarPackets;
using SuTengScanMsgPtr = boost::shared_ptr<SuTengScanMsg>;
using SuTengPacketsMsgPtr = boost::shared_ptr<SuTengPacketsMsg>;
using SuTengIRADPacketsMsgPtr = boost::shared_ptr<SuTengIRADPacketsMsg>;

using RSPoint = robosense::lidar::Manager::DriverPointType;
using RSCloudType = pcl::PointCloud<RSPoint>;

using IRADPoint = avos::driver::PointXYZIT;
using IRADCloudType = pcl::PointCloud<IRADPoint>;

// 禾赛
using HeSaiScanMsg = hesai_lidar::HesaiScan;
using HeSaiScanMsgPtr = boost::shared_ptr<HeSaiScanMsg>;

#endif  // MAPPING_MESSAGE_DEF_H
