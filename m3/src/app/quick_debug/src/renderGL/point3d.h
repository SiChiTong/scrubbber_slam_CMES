//
// Created by gaoxiang on 2019/12/18.
//

#ifndef HAMO_LINUX_POINT3D_H
#define HAMO_LINUX_POINT3D_H

#include <cmath>
#include <cstring>
#include <vector>

namespace HAMO {

class Point3d {
   public:
    //! \brief
    double x = 0;
    //! \brief
    double y = 0;
    //! \brief
    double z = 0;

   public:
    //! \brief
    Point3d();

    //! \brief
    ~Point3d();

    Point3d(double xx, double yy);

    //! \brief  带参数构造函数
    Point3d(double xx, double yy, double zz);

    static double Distance(const Point3d &p1, const Point3d &p2) {
        double dlength2 = (p1.x - p2.x) * (p1.x - p2.x) +
                          (p1.y - p2.y) * (p1.y - p2.y) +
                          (p1.z - p2.z) * (p1.z - p2.z);
        double disance = sqrt(dlength2);

        return disance;
    }

    static Point3d MiddlePoint(const Point3d &p1, const Point3d &p2) {
        Point3d pnt;
        pnt.x = (p1.x + p2.x) * 0.5;
        pnt.y = (p1.y + p2.y) * 0.5;
        pnt.z = (p1.z + p2.z) * 0.5;
        return pnt;
    }

   public:
    Point3d &operator=(const Point3d &s);

    Point3d &operator+=(const Point3d &p);

    Point3d &operator-=(const Point3d &p);

    Point3d &operator*=(double c);

    Point3d &operator/=(double c);

    friend const Point3d operator-(const Point3d &p1, const Point3d &p2);

    bool operator==(const Point3d &) const;

    bool operator!=(const Point3d &) const;
};

struct TrackPoint {
    Point3d pnt;
    double ndt = 0;
    int img = 0;
};

struct CurbsTrack {
    std::vector<TrackPoint> trackSet;
};

}  // namespace HAMO

#endif  // HAMO_LINUX_POINT3D_H
