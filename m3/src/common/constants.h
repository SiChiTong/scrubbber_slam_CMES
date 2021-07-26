//
// Created by gaoxiang on 2020/8/10.
//

#ifndef MAPPING_CONSTANTS_H
#define MAPPING_CONSTANTS_H

/// 常量定义
namespace mapping::common {

constexpr double KSMA = 6378137.0;
constexpr double KSMB = 6356752.31425;
constexpr double KUTMSCALEFACTOR = 0.9996;
constexpr double KSINSRADTODEG = 57.295779513082323;
constexpr double KSINSDEGTORAD = 0.017453292519943;
constexpr double KSINSR0 = 6378137.0;
constexpr double KSINSE = 0.08181919108425;
constexpr double KSINSE2 = 0.00669437999013;

constexpr double kWGS84_RE = 6378137.0;
constexpr double kWGS84_RA = 6378140.0;
constexpr double kWGS84_RB = 6356755.3;
constexpr double kWGS84_IF = 298.257223563;
constexpr double kWGS84_F = 0.003352810664747;  /// WGS84_F = 1/WGS84_IF
constexpr double kOMEGA_IE = 7.2921151467e-5;

constexpr double kGRAVITY = 9.80665;                 /// g
constexpr double kM_GRAVITY = kGRAVITY / 1000.0;     /// mg
constexpr double kU_GRAVITY = kGRAVITY / 1000000.0;  /// ug

constexpr double kPI = 3.141592653589793;        /// pi
constexpr double kPI_2 = kPI / 2.0;              /// pi/2
constexpr double kRAD2DEG = 57.295779513082323;  /// rad -> deg
constexpr double kDEG2RAD = 0.017453292519943;   /// deg -> rad

constexpr float GOOD_GPS_POS_NOISE = 0.15;
constexpr float BAD_GPS_POS_NOISE = 50.0;
constexpr float NO_GPS_POS_NOISE = 1e8;

constexpr float GOOD_GPS_ANGLE_NOISE = 0.08;
constexpr float BAD_GPS_ANGLE_NOISE = 1e3;
constexpr float NO_GPS_ANGLE_NOISE = 3e9;

}  // namespace mapping::common

#endif  // MAPPING_CONSTANTS_H
