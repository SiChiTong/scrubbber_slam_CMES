//
// Created by gaoxiang on 2020/5/15.
//

#ifndef SCRUBBER_SLAM_DR_CONSTANT_H
#define SCRUBBER_SLAM_DR_CONSTANT_H

#include "common/num_types.h"

constexpr double kStaticTime = 1;                // static time 3s
constexpr double kRAD2DEG = 57.295779513082323;  /// rad -> deg
constexpr double kDEG2RAD = 0.017453292519943;   /// deg -> rad

/// 重力
constexpr double kGRAVITY = 9.80665;                 /// g
constexpr double kM_GRAVITY = kGRAVITY / 1000.0;     /// mg
constexpr double kU_GRAVITY = kGRAVITY / 1000000.0;  /// ug

/// 轮子
const double WHEEL_RADIUS = 0.156;
const double CIRCLE_PULSE = 1024.0;

/// 频率和频率倒数
const double fIMU_FREQ = 100;
const double fGNSS_FREQ = 10;
const double fODOM_FREQ = 10;
const double fLIDAR_FREQ = 10;

const double tIMU_SPAN = 1.0 / fIMU_FREQ;
const double tGNSS_SPAN = 1.0 / fGNSS_FREQ;
const double tODOM_SPAN = 1.0 / fODOM_FREQ;
const double tLIDAR_SPAN = 1.0 / fLIDAR_FREQ;

/// 静态阈值
const double STATIC_VEL_THESHOLD = 0.1;
const double MOTION_VEL_THESHOLD = 2;

/// WGS84参数
const double kWGS84_RE = 6378137.0;
const double kWGS84_F = 0.003352810664747;  /// WGS84_F = 1/WGS84_IF
const double kOMEGA_IE = 7.2921151467e-5;

/// 功能函数
inline double SINL(double lat) { return sin(lat * kDEG2RAD); }
inline double COSL(double lat) { return cos(lat * kDEG2RAD); }
inline double TANL(double lat) { return tan(lat * kDEG2RAD); }
inline double SIN2L(double lat) { return sin(2.0 * lat * kDEG2RAD); }

inline double SQ(double x) { return (x * x); }
inline bool StaticCheck(double x, double y) { return fabs(x) <= STATIC_VEL_THESHOLD && fabs(y) <= STATIC_VEL_THESHOLD; }
inline double RM(double lat) { return (kWGS84_RE * (1 - 2.0 * kWGS84_F + 3 * kWGS84_F * SQ(SINL(lat)))); }
inline double RN(double lat) { return (kWGS84_RE * (1 + kWGS84_F * SQ(SINL(lat)))); }

#endif  // SCRUBBER_SLAM_DR_CONSTANT_H
