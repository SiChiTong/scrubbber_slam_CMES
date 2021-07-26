//
// Created by gaoxiang on 2019/12/12.
//

#ifndef HAMO_LINUX_EULER_H
#define HAMO_LINUX_EULER_H

#include "algorithm/common.h"
#include "algorithm/matrix44.h"

namespace HAMO {

template <typename N>
class Euler {
   public:
    typedef N ValueType;
    typedef Matrix4x4<N> M44Type;
    typedef Eigen::Quaternion<N> QuatType;

   public:
    Euler();

    Euler(N x, N y, N z);

    Euler(const Euler<N> &rhs);

   public:
    void Set(N x, N y, N z);

    void FromQuaternion(const QuatType &quat);

    M44Type MakeMatrix() const;

    QuatType MakeQuaternion() const;

   public:
    N x;
    N y;
    N z;
};

//------------------------------------------------------------------------------
typedef Euler<float> Eulerf;
typedef Euler<double> Eulerd;

//------------------------------------------------------------------------------

template <typename N>
Euler<N>::Euler() : x(0), y(0), z(0) {}

template <typename N>
Euler<N>::Euler(N xx, N yy, N zz) : x(xx), y(yy), z(zz) {}

template <typename N>
Euler<N>::Euler(const Euler<N> &rhs) {
    x = rhs.x;
    y = rhs.y;
    z = rhs.z;
}

template <typename N>
void Euler<N>::Set(N xx, N yy, N zz) {
    x = xx;
    y = yy;
    z = zz;
}

template <typename N>
void Euler<N>::FromQuaternion(const QuatType &quat) {
    // XYZ-order
    N a = 2 * (quat.w * quat.x - quat.y * quat.z);
    N b = 2 * (quat.w * quat.y + quat.x * quat.z);
    N c = 2 * (quat.w * quat.z - quat.x * quat.y);

    N d = 2 * (quat.x * quat.x + quat.y * quat.y);
    N e = 2 * (quat.z * quat.z + quat.y * quat.y);

    x = atan2(a, 1 - d);
    y = asin(b);
    z = atan2(c, 1 - e);
}

template <typename N>
Matrix4x4<N> Euler<N>::MakeMatrix() const {
    M44Type mat;

    N a = cos(x), b = sin(x);
    N c = cos(y), d = sin(y);
    N e = cos(z), f = sin(z);

    N ae = a * e, af = a * f, be = b * e, bf = b * f;

    mat.m11 = c * e;
    mat.m12 = af + be * d;
    mat.m13 = bf - ae * d;

    mat.m21 = -c * f;
    mat.m22 = ae - bf * d;
    mat.m23 = be + af * d;

    mat.m31 = d;
    mat.m32 = -b * c;
    mat.m33 = a * c;

    return mat;
}

template <typename N>
Eigen::Quaternion<N> Euler<N>::MakeQuaternion() const {
    N c1 = cos(x / 2);
    N c2 = cos(y / 2);
    N c3 = cos(z / 2);

    N s1 = sin(x / 2);
    N s2 = sin(y / 2);
    N s3 = sin(z / 2);

    return Quaternion<N>(c1 * c2 * c3 - s1 * s2 * s3,   // w
                         s1 * c2 * c3 + c1 * s2 * s3,   // x
                         c1 * s2 * c3 - s1 * c2 * s3,   // y
                         c1 * c2 * s3 + s1 * s2 * c3);  // z
}

}  // namespace HAMO

#endif  // HAMO_LINUX_EULER_H
