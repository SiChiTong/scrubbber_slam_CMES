//
// Created by gaoxiang on 2019/12/18.
//

#include "renderGL/point3d.h"

namespace HAMO {

Point3d::Point3d() : x(0.0), y(0.0), z(0.0) {}
Point3d::Point3d(double xx, double yy) : x(xx), y(yy), z(0.0) {}
Point3d::Point3d(double xx, double yy, double zz) {
    x = xx;
    y = yy;
    z = zz;
}

Point3d::~Point3d() {}

Point3d &Point3d::operator=(const Point3d &p1) {
    this->x = p1.x;
    this->y = p1.y;
    this->z = p1.z;

    return *this;
}

Point3d &Point3d::operator+=(const Point3d &p1) {
    this->x += p1.x;
    this->y += p1.y;
    this->z += p1.z;

    return *this;
}

Point3d &Point3d::operator-=(const Point3d &p1) {
    this->x -= p1.x;
    this->y -= p1.y;
    this->z -= p1.z;

    return *this;
}

Point3d &Point3d::operator*=(double c) {
    this->x *= c;
    this->y *= c;
    this->z *= c;

    return *this;
}

Point3d &Point3d::operator/=(double c) {
    this->x /= c;
    this->y /= c;
    this->z /= c;

    return *this;
}

Point3d const operator-(const Point3d &p1, const Point3d &p2) {
    return Point3d(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

bool Point3d::operator==(const Point3d &right) const {
    if (x == right.x && y == right.y && z == right.z) return true;
    else
        return false;
}

bool Point3d::operator!=(const Point3d &right) const {
    if (x == right.x && y == right.y && z == right.z) return false;
    else
        return true;
}

}  // namespace HAMO
