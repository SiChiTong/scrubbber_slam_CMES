//
// Created by gaoxiang on 2020/8/13.
//

#ifndef MAPPING_GPS_TRANS_H
#define MAPPING_GPS_TRANS_H

#include "common/num_type.h"
#include "common/origin_point_info.h"
#include "common/std_headers.h"
#include "common/utm_coordinates.h"
#include "common/wgs84_coordinates.h"

#include "common/message_def.h"

namespace mapping::core {

/// 坐标转换类(GPS相关)
/// TODO 这个还是有点乱，static类型不使用局部变量
class GpsTransform {
   public:
    /// 非数据准入阶段使用
    explicit GpsTransform(V3d origin, double ant_x, double ant_y, double ant_angle)
        : map_origin_(origin), antenna_pose_(ant_x, ant_y, ant_angle) {}

    /// 获取地图平移量时，使用此构造函数
    explicit GpsTransform(double ant_x, double ant_y, double ant_angle) : antenna_pose_(ant_x, ant_y, ant_angle) {}

    /// 将gps的值转到后轴中心
    static V2d GpsFromTranslation(const GpsMsg &gps, const V3d &delta);

    /// 将gps_msg转换成utm坐标(x,y,a)
    V3d GpsToXYA(const GpsMsg &gps);

    /// 将gps_msg转换成utm坐标(x,y,z,a)
    V4d GpsToXYZA(const GpsMsg &gps);

    /// 将gps_msg转换成utm坐标(x,y,z,zone,a)
    void GpsToUtmXYZAAndZone(const GpsMsg &gps, V3d &xyz, int &zone, double &angle);

    /// gps_msg转换成矩阵形式
    common::OriginPointInformation GpsToOriginInfo(const GpsMsg &gps);

    /// 经纬高转换为原点信息
    common::OriginPointInformation LatLonToOriginInfo(double lon, double lat, double height);

    /// GPS 转矩阵
    void GpsToEigen(const GpsMsg &gps, M4d &eigen_pose);

    /// 将航向转换为角度
    double Heading2Angle(double input);

    /// 将角度转换为航向
    double Angle2Heading(double input);

    /// 经纬坐标转换成utm的xy坐标
    static V2d LatLonToUtmXY(double lon, double lat);

    /// utm坐标反算为经纬度
    static V2d UtmXYToLatLon(double x, double y, int zone, bool southhemi);

   private:
    static V2d GpsFromTranslation(const V4d &origin, const V3d &delta);

    static void XYZToEigen(const V4d &gps_pose, M4d &eigen_pose);

    static double Arclength0fMeridian(const double phi);

    static double UtmCentralMeridian(int zone);

    static double FootpointLatitude(const double y);

    static void MapLatLonToXY(double phi, double lambda, double lambda0, common::UTMPoint &xy);

    static void MapXYToLatLon(double x, double y, double lambda0, common::WGS84Corr &philambda);

    static void LatLonToUtmXY(double lon_rad, double lat_rad, common::UTMPoint &xy);

    static void UtmXYToLatLon(double x, double y, int zone, bool southhemi, common::WGS84Corr &latlon);

    static void XYZToBLH(const V3d &xyz, V3d &blh);

    static void BLHToXYZ(const V3d &blh, V3d &xyz);

   private:
    V3d map_origin_ = V3d::Zero();
    V3d antenna_pose_ = V3d::Zero();
};
}  // namespace mapping::core

#endif  // MAPPING_GPS_TRANS_H
