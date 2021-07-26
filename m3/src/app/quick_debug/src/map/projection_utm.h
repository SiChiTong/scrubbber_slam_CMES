//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_PROJECTION_UTM_H
#define HAMO_LINUX_PROJECTION_UTM_H

#include "map/map_define.h"

namespace HAMO {

class ProjectionUTM {
   public:
    void CartesianToLatLon(double x, double y, int zone, bool southhemi,
                           LatLon &latlon);

    void LatLonToCartesian(double lat, double lon, UTMPoint &xy);

   private:
    void MapXYToLatLon(double x, double y, double lambda0, LatLon &philambda);

    void MapLatLonToXY(double phi, double lambda, double lambda0, UTMPoint &xy);

    double ArcLengthOfMeridian(double phi);

    double UTMCentralMeridian(int zone);

    double FootpointLatitude(double y);

   private:
    static double pi;

    static double sm_a;
    static double sm_b;
    static double sm_EccSquared;
    static double UTMScaleFactor;

   public:
    static int zone;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_PROJECTION_UTM_H
