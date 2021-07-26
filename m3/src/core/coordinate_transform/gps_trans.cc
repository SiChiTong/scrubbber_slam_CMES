//
// Created by gaoxiang on 2020/8/13.
//
#include "core/coordinate_transform/gps_trans.h"
#include "common/constants.h"

#include <glog/logging.h>
#include <cmath>

namespace mapping {
namespace core {

using namespace mapping::common;

V2d GpsTransform::GpsFromTranslation(const GpsMsg &gps, const V3d &delta) {
    return GpsFromTranslation(V4d(gps.lon, gps.lat, gps.height, gps.heading), delta);
}

V2d GpsTransform::GpsFromTranslation(const V4d &origin, const V3d &delta) {
    const double E = 1.0 / 298.257;
    const double Re = 6378137.0;

    double originLatRad = origin[1] * KSINSDEGTORAD;
    double originLonRad = origin[0] * KSINSDEGTORAD;
    double antenna_distance = std::hypot(delta[0], delta[1]);
    double bearingDeg = origin[3] + delta[2] - std::atan2(-delta[1], -delta[0]) * KSINSRADTODEG;
    if (bearingDeg > 180) {
        bearingDeg -= 360;
    } else if (bearingDeg < -180) {
        bearingDeg += 360;
    }

    double bearingRad = bearingDeg * KSINSDEGTORAD;

    double Rm = Re * (1 - 2 * E + 3 * E * sin(originLatRad) * sin(originLatRad));
    double Rn = Re * (1 + E * sin(originLatRad) * sin(originLatRad));

    double Rx = antenna_distance * sin(bearingRad);
    double Ry = antenna_distance * cos(bearingRad);

    return V2d((originLonRad + Rx / ((Rn + origin[2]) * cos(originLatRad))) * KSINSRADTODEG,
               (originLatRad + Ry / (Rm + origin[2])) * KSINSRADTODEG);
}

V3d GpsTransform::GpsToXYA(const GpsMsg &gps) {
    auto lidar_position = GpsFromTranslation(gps, antenna_pose_);
    auto utm_pose = LatLonToUtmXY(lidar_position[0], lidar_position[1]);
    return V3d(utm_pose[0] - map_origin_[0], utm_pose[1] - map_origin_[1],
               Heading2Angle(gps.heading + antenna_pose_[2]));
}

void GpsTransform::GpsToUtmXYZAAndZone(const GpsMsg &gps, V3d &xyz, int &zone, double &angle) {
    zone = static_cast<int>((gps.lon + 180) / 6) + 1;
    auto car_postion_gps = GpsFromTranslation(gps, antenna_pose_);
    auto utm_pose = LatLonToUtmXY(car_postion_gps[0], car_postion_gps[1]);
    xyz[0] = utm_pose[0];
    xyz[1] = utm_pose[1];
    xyz[2] = gps.height;
    angle = Heading2Angle(gps.heading + antenna_pose_[2]);
}

V4d GpsTransform::GpsToXYZA(const GpsMsg &gps) {
    auto utm_pose_2d = GpsToXYA(gps);
    return V4d(utm_pose_2d[0], utm_pose_2d[1], gps.height - map_origin_[2], utm_pose_2d[2]);
}

OriginPointInformation GpsTransform::GpsToOriginInfo(const GpsMsg &gps) {
    OriginPointInformation result;
    V3d xyz;
    double angle;
    GpsToUtmXYZAAndZone(gps, xyz, result.map_origin_zone, angle);
    result.map_origin_x = xyz[0];
    result.map_origin_y = xyz[1];
    result.map_origin_z = xyz[2];
    if (gps.lat < 0) {
        result.is_southern = true;
    } else {
        result.is_southern = false;
    }
    return result;
}

OriginPointInformation GpsTransform::LatLonToOriginInfo(double lon, double lat, double height) {
    OriginPointInformation result;
    GpsMsg gps;
    gps.lon = lon;
    gps.lat = lat;
    gps.height = height;
    gps.heading = 0;

    V3d xyz;
    int zone;
    double angle;
    GpsToUtmXYZAAndZone(gps, xyz, zone, angle);
    result.map_origin_x = xyz[0];
    result.map_origin_y = xyz[1];
    result.map_origin_z = xyz[2];
    result.map_origin_zone = zone;

    if (gps.lat < 0) {
        result.is_southern = true;
    } else {
        result.is_southern = false;
    }
    return result;
}

void GpsTransform::XYZToEigen(const V4d &gps_pose, M4d &eigen_pose) {
    double gps_x = gps_pose[0];
    double gps_y = gps_pose[1];
    double gps_z = gps_pose[2];
    double gps_roll = 0;
    double gps_pitch = 0;
    double gps_yaw = gps_pose[3] * KSINSDEGTORAD;

    Eigen::AngleAxisd gps_rotation_x(gps_roll, V3d::UnitX());
    Eigen::AngleAxisd gps_rotation_y(gps_pitch, V3d::UnitY());
    Eigen::AngleAxisd gps_rotation_z(gps_yaw, V3d::UnitZ());
    Eigen::Translation<double, 3> gps_translation_3d(gps_x, gps_y, gps_z);
    eigen_pose = (gps_translation_3d * gps_rotation_z * gps_rotation_y * gps_rotation_x).matrix();
}

void GpsTransform::GpsToEigen(const GpsMsg &gps, M4d &eigen_pose) { XYZToEigen(GpsToXYZA(gps), eigen_pose); }

double GpsTransform::Arclength0fMeridian(const double phi) {
    /* Precalculate n */
    double n = (KSMA - KSMB) / (KSMA + KSMB);

    /* Precalculate alpha */
    double alpha = ((KSMA + KSMB) / 2.0) * (1.0 + (pow(n, 2.0) / 4.0) + (pow(n, 4.0) / 64.0));

    /* Precalculate beta */
    double beta = (-3.0 * n / 2.0) + (9.0 * pow(n, 3.0) / 16.0) + (-3.0 * pow(n, 5.0) / 32.0);

    /* Precalculate gamma */
    double gamma = (15.0 * pow(n, 2.0) / 16.0) + (-15.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta */
    double delta = (-35.0 * pow(n, 3.0) / 48.0) + (105.0 * pow(n, 5.0) / 256.0);

    /* Precalculate epsilon */
    double epsilon = (315.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series and return */
    double result = alpha * (phi + (beta * sin(2.0 * phi)) + (gamma * sin(4.0 * phi)) + (delta * sin(6.0 * phi)) +
                             (epsilon * sin(8.0 * phi)));

    return result;
}

double GpsTransform::UtmCentralMeridian(int zone) { return (-183.0 + (zone * 6.0)) * KSINSDEGTORAD; }

double GpsTransform::FootpointLatitude(const double y) {
    /* Precalculate n (Eq. 10.18) */
    double n = (KSMA - KSMB) / (KSMA + KSMB);

    /* Precalculate alpha (Eq. 10.22) */
    /* (Same as alpha in Eq. 10.17) */
    double alpha = ((KSMA + KSMB) / 2.0) * (1 + (pow(n, 2.0) / 4) + (pow(n, 4.0) / 64));

    /* Precalculate yy (Eq. 10.23) */
    double yy = y / alpha;

    /* Precalculate beta (Eq. 10.22) */
    double beta = (3.0 * n / 2.0) + (-27.0 * pow(n, 3.0) / 32.0) + (269.0 * pow(n, 5.0) / 512.0);

    /* Precalculate gamma (Eq. 10.22) */
    double gamma = (21.0 * pow(n, 2.0) / 16.0) + (-55.0 * pow(n, 4.0) / 32.0);

    /* Precalculate delta (Eq. 10.22) */
    double delta = (151.0 * pow(n, 3.0) / 96.0) + (-417.0 * pow(n, 5.0) / 128.0);

    /* Precalculate epsilon (Eq. 10.22) */
    double epsilon = (1097.0 * pow(n, 4.0) / 512.0);

    /* Now calculate the sum of the series (Eq. 10.21) */
    double result =
        yy + (beta * sin(2.0 * yy)) + (gamma * sin(4.0 * yy)) + (delta * sin(6.0 * yy)) + (epsilon * sin(8.0 * yy));

    return result;
}

void GpsTransform::MapLatLonToXY(const double phi, const double lambda, double lambda0, UTMPoint &xy) {
    /* Precalculate ep2 */
    double ep2 = (pow(KSMA, 2.0) - pow(KSMB, 2.0)) / pow(KSMB, 2.0);

    /* Precalculate nu2 */
    double nu2 = ep2 * pow(cos(phi), 2.0);

    /* Precalculate nn */
    double nn = pow(KSMA, 2.0) / (KSMB * sqrt(1 + nu2));

    /* Precalculate t */
    double t = tan(phi);
    double t2 = t * t;
    // double tmp = (t2 * t2 * t2) - pow(t, 6.0);

    /* Precalculate l */
    double l = lambda - lambda0;

    /* Precalculate coefficients for l**nn in the equations below
    so a normal human being can read the expressions for easting
    and northing
    -- l**1 and l**2 have coefficients of 1.0 */
    double l3coef = 1.0 - t2 + nu2;

    double l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

    double l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2 - 58.0 * t2 * nu2;

    double l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2 - 330.0 * t2 * nu2;

    double l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

    double l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

    /* Calculate easting (x) */
    xy.x = nn * cos(phi) * l + (nn / 6.0 * pow(cos(phi), 3.0) * l3coef * pow(l, 3.0)) +
           (nn / 120.0 * pow(cos(phi), 5.0) * l5coef * pow(l, 5.0)) +
           (nn / 5040.0 * pow(cos(phi), 7.0) * l7coef * pow(l, 7.0));

    /* Calculate northing (y) */
    xy.y = Arclength0fMeridian(phi) + (t / 2.0 * nn * pow(cos(phi), 2.0) * pow(l, 2.0)) +
           (t / 24.0 * nn * pow(cos(phi), 4.0) * l4coef * pow(l, 4.0)) +
           (t / 720.0 * nn * pow(cos(phi), 6.0) * l6coef * pow(l, 6.0)) +
           (t / 40320.0 * nn * pow(cos(phi), 8.0) * l8coef * pow(l, 8.0));
    return;
}

void GpsTransform::MapXYToLatLon(double x, double y, double lambda0, WGS84Corr &philambda) {
    /* Get the value of phif, the footpoint latitude. */
    double phif = FootpointLatitude(y);

    /* Precalculate ep2 */
    double ep2 = (pow(KSMA, 2.0) - pow(KSMB, 2.0)) / pow(KSMB, 2.0);

    /* Precalculate cos (phif) */
    double cf = cos(phif);

    /* Precalculate nuf2 */
    double nuf2 = ep2 * pow(cf, 2.0);

    /* Precalculate nf and initialize nfkDEG2RADpow */
    double nf = pow(KSMA, 2.0) / (KSMB * sqrt(1 + nuf2));
    double nfpow = nf;

    /* Precalculate tf */
    double tf = tan(phif);
    double tf2 = tf * tf;
    double tf4 = tf2 * tf2;

    /* Precalculate fractional coefficients for x**n in the equations
    below to simplify the expressions for latitude and longitude. */
    double x1frac = 1.0 / (nfpow * cf);

    nfpow *= nf; /* now equals nf**2) */
    double x2frac = tf / (2.0 * nfpow);

    nfpow *= nf; /* now equals nf**3) */
    double x3frac = 1.0 / (6.0 * nfpow * cf);

    nfpow *= nf; /* now equals nf**4) */
    double x4frac = tf / (24.0 * nfpow);

    nfpow *= nf; /* now equals nf**5) */
    double x5frac = 1.0 / (120.0 * nfpow * cf);

    nfpow *= nf; /* now equals nf**6) */
    double x6frac = tf / (720.0 * nfpow);

    nfpow *= nf; /* now equals nf**7) */
    double x7frac = 1.0 / (5040.0 * nfpow * cf);

    nfpow *= nf; /* now equals nf**8) */
    double x8frac = tf / (40320.0 * nfpow);

    /* Precalculate polynomial coefficients for x**n.
    -- x**1 does not have a polynomial coefficient. */
    double x2poly = -1.0 - nuf2;

    double x3poly = -1.0 - 2 * tf2 - nuf2;

    double x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2 - 3.0 * (nuf2 * nuf2) - 9.0 * tf2 * (nuf2 * nuf2);

    double x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;

    double x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2 + 162.0 * tf2 * nuf2;

    double x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);

    double x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);

    /* Calculate latitude */
    philambda.lat = phif + x2frac * x2poly * (x * x) + x4frac * x4poly * pow(x, 4.0) + x6frac * x6poly * pow(x, 6.0) +
                    x8frac * x8poly * pow(x, 8.0);

    /* Calculate longitude */
    philambda.lon = lambda0 + x1frac * x + x3frac * x3poly * pow(x, 3.0) + x5frac * x5poly * pow(x, 5.0) +
                    x7frac * x7poly * pow(x, 7.0);
    return;
}

void GpsTransform::LatLonToUtmXY(double lon_rad, double lat_rad, UTMPoint &xy) {
    int zone = 0;
    zone = static_cast<int>((lon_rad * KSINSRADTODEG + 180) / 6) + 1;

    MapLatLonToXY(lat_rad, lon_rad, UtmCentralMeridian(zone), xy);

    /* Adjust easting and northing for UTM system. */
    xy.x = xy.x * KUTMSCALEFACTOR + 500000.0;
    xy.y = xy.y * KUTMSCALEFACTOR;
    if (xy.y < 0.0) {
        xy.y += 10000000.0;
    }
}

void GpsTransform::UtmXYToLatLon(double x, double y, int zone, bool southhemi, WGS84Corr &latlon) {
    double xx = x;
    xx -= 500000.0;
    xx /= KUTMSCALEFACTOR;

    /* If in southern hemisphere, adjust y accordingly. */
    double yy = y;
    if (southhemi) {
        yy -= 10000000.0;
    }
    yy /= KUTMSCALEFACTOR;

    double cmeridian = UtmCentralMeridian(zone);
    MapXYToLatLon(xx, yy, cmeridian, latlon);
}

void GpsTransform::XYZToBLH(const V3d &xyz, V3d &blh) {
    double r2 = xyz[0] * xyz[0] + xyz[1] * xyz[1];
    double z = 0.0;
    double zk = 0.0;
    double v = KSINSR0;

    for (z = xyz[2], zk = 0.0; fabs(z - zk) >= 1E-4;) {
        zk = z;
        const double sinp = z / sqrt(r2 + z * z);
        v = KSINSR0 / sqrt(1.0 - KSINSE2 * sinp * sinp);
        z = xyz[2] + v * KSINSE2 * sinp;
    }

    blh[0] = r2 > 1E-12 ? atan2(xyz[1], xyz[0]) : 0.0;
    blh[1] = r2 > 1E-12 ? atan(z / sqrt(r2)) : (xyz[2] > 0.0 ? M_PI / 2.0 : -M_PI / 2.0);
    blh[2] = sqrt(r2 + z * z) - v;
}

void GpsTransform::BLHToXYZ(const V3d &blh, V3d &xyz) {
    double sin_lati_2 = sin(blh[1]) * sin(blh[1]);
    double temp_a = sqrt(1.0 - KSINSE2 * sin_lati_2);
    double rn = KSINSR0 / temp_a;

    double cos_lat = cos(blh[1]);
    double sin_lat = sin(blh[1]);
    double cos_long = cos(blh[0]);
    double sin_long = sin(blh[0]);

    xyz[0] = (rn + blh[2]) * cos_lat * cos_long;
    xyz[1] = (rn + blh[2]) * cos_lat * sin_long;
    xyz[2] = ((1 - KSINSE2) * rn + blh[2]) * sin_lat;
}

V2d GpsTransform::LatLonToUtmXY(double lon, double lat) {
    UTMPoint xy;
    LatLonToUtmXY(lon * KSINSDEGTORAD, lat * KSINSDEGTORAD, xy);
    return V2d(xy.x, xy.y);
}

V2d GpsTransform::UtmXYToLatLon(double x, double y, int zone, bool southhemi) {
    WGS84Corr latlon;
    UtmXYToLatLon(x, y, zone, southhemi, latlon);
    return V2d(latlon.lat * KSINSRADTODEG, latlon.lon * KSINSRADTODEG);
}

double GpsTransform::Heading2Angle(double input) {
    double output = 90 - input;
    if (output < -180) output += 360;
    if (output > 180) output -= 360;
    return output;
}

double GpsTransform::Angle2Heading(double input) {
    double output = 90 - input;
    if (output < 0) output += 360;
    if (output > 360) output -= 360;
    return output;
}

}  // namespace core
}  // namespace mapping