//
// Created by xiang on 2020/1/9.
//

#ifndef HAMO_LINUX_QUATERNION_H
#define HAMO_LINUX_QUATERNION_H

#include "algorithm/vector3.h"

namespace HAMO {

template <typename N>
class Quaternion {
   public:
    typedef N ValueType;

   public:
    Quaternion();

    Quaternion(const Quaternion<N> &other);

    Quaternion(N w, N x, N y, N z);

   public:
    void Set(N w, N x, N y, N z);

    N Length(void) const;

    N LengthSquare(void) const;

    N squaredNorm() const { return LengthSquare(); }

    N Normalize(void);

    void FromAngleAxis(const Vector3<N> &rkAxis, float rfAngle);

    Quaternion<N> Inverse() const;

    Quaternion<N> inverse() const { return Inverse(); }

    Quaternion<N> Conjugate() const;

    Quaternion<N> CrossProduct(const Quaternion<N> &rkQ) const;

    N DotProduct(const Quaternion<N> &rkQ) const;

    Vector3<N> Multiply(const Vector3<N> &vec) const;

    // 重载操作符
    Quaternion<N> operator-() const;

   public:
    N x, y, z;
    N w;
};

//------------------------------------------------------------------------------
typedef Quaternion<float> Quaternionf;
typedef Quaternion<double> Quaterniond;

//------------------------------------------------------------------------------

template <typename N>
Quaternion<N>::Quaternion() {
    w = ValueType(1);
    x = y = z = ValueType(0);
}

template <typename N>
Quaternion<N>::Quaternion(const Quaternion<N> &other) {
    w = other.w;
    x = other.x;
    y = other.y;
    z = other.z;
}

template <typename N>
Quaternion<N>::Quaternion(N ww, N xx, N yy, N zz) {
    w = ww;
    x = xx;
    y = yy;
    z = zz;
}

template <typename N>
void Quaternion<N>::Set(N ww, N xx, N yy, N zz) {
    w = ww;
    x = xx;
    y = yy;
    z = zz;
}

template <typename N>
N Quaternion<N>::Length() const {
    return Math<N>::Sqrt(x * x + y * y + z * z + w * w);
}

template <typename N>
N Quaternion<N>::LengthSquare() const {
    return x * x + y * y + z * z + w * w;
}

template <typename N>
N Quaternion<N>::Normalize() {
    N fLength = Length();
    if (fLength > ValueType(0)) {
        N fInvLength = ValueType(1) / fLength;
        x *= fLength;
        y *= fLength;
        z *= fLength;
        w *= fLength;
    } else {
        x = y = z = w = ValueType(0);
    }
    return fLength;
}

template <typename N>
Quaternion<N> Quaternion<N>::Conjugate() const {
    return Quaternion<N>(w, -x, -y, -z);
}

// this*rkq 标准乘法
template <typename N>
Quaternion<N> Quaternion<N>::CrossProduct(const Quaternion<N> &rkQ) const {
    return Quaternion<N>(w * rkQ.w - x * rkQ.x - y * rkQ.y - z * rkQ.z,
                         w * rkQ.x + x * rkQ.w + y * rkQ.z - z * rkQ.y,
                         w * rkQ.y + y * rkQ.w + z * rkQ.x - x * rkQ.z,
                         w * rkQ.z + z * rkQ.w + x * rkQ.y - y * rkQ.x);
}

template <typename N>
N Quaternion<N>::DotProduct(const Quaternion<N> &rkQ) const {
    return w * rkQ.w + x * rkQ.x + y * rkQ.y + z * rkQ.z;
}

template <typename N>
Vector3<N> Quaternion<N>::Multiply(const Vector3<N> &vec) const {
    Vector3<N> qvec(x, y, z);

    Vector3<N> uv = qvec.CrossProduct(vec);
    Vector3<N> uuv = qvec.CrossProduct(uv);

    uv.Scale(2.0f * w);
    uuv.Scale(2.0f);

    return vec + uv + uuv;
}

// rkAxis轴， rfAngle 弧度制
template <typename N>
void Quaternion<N>::FromAngleAxis(const Vector3<N> &rkAxis, float rfAngle) {
    const N epsilon = N(0.0000001);

    N length = rkAxis.Length();
    if (length < epsilon) {
        *this = Quaternion<N>();
    } else {
        N fHalfAngle = ValueType(rfAngle * 0.5);

        N fInv = N(1.0) / length;
        N fSin = Math<N>::Sin(fHalfAngle);
        N fCos = Math<N>::Cos(fHalfAngle);

        w = fCos;
        x = fSin * rkAxis.x * fInv;
        y = fSin * rkAxis.y * fInv;
        z = fSin * rkAxis.z * fInv;
    }
}

template <typename N>
Quaternion<N> Quaternion<N>::Inverse() const {
    N length2 = x * x + y * y + z * z + w * w;

    N fInv = N(1) / length2;

    return Quaternion<N>(w * fInv, -x * fInv, -y * fInv, -z * fInv);
}

/*
template <typename N>
Matrix4x4f  Quaternion<N>::ToMatrix4x4()
{

N x2 = x * x;
N y2 = y * y;
N z2 = z * z;
N xy = x * y;
N xz = x * z;
N yz = y * z;
N wx = w * x;
N wy = w * y;
N wz = w * z;

return Matrix4x4<N>( 1.0f - 2.0f * (y2 + z2), 2.0f * (xy + wz),         2.0f *
(xz - wy),        0.0f, 2.0f * (xy - wz),        1.0f - 2.0f * (x2 + z2),  2.0f
* (yz + wx),        0.0f, 2.0f * (xz + wy),        2.0f * (yz -
wx),         1.0f - 2.0f * (x2 + y2), 0.0f, 0.0f,                    0.0f, 0.0f,
1.0f);
}
*/

template <typename N>
Quaternion<N> Quaternion<N>::operator-() const {
    return Quaternion(-w, -x, -y, -z);
}

//////////////////////////////////////////////////////////////////////////

template <typename N>
Quaternion<N> operator+(const Quaternion<N> &q1, const Quaternion<N> &q2) {
    return Quaternion<N>(q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z);
}

template <typename N>
Quaternion<N> operator-(const Quaternion<N> &q1, const Quaternion<N> &q2) {
    return Quaternion<N>(q1.w - q2.w, q1.x - q2.x, q1.y - q2.y, q1.z - q2.z);
}

// q1 * q2乘法，非标准定义，和标准乘法定义顺序相反
template <typename N>
Quaternion<N> operator*(const Quaternion<N> &q1, const Quaternion<N> &q2) {
    return Quaternion<N>(q2.w * q1.w - q2.x * q1.x - q2.y * q1.y - q2.z * q1.z,
                         q2.w * q1.x + q2.x * q1.w + q2.y * q1.z - q2.z * q1.y,
                         q2.w * q1.y + q2.y * q1.w + q2.z * q1.x - q2.x * q1.z,
                         q2.w * q1.z + q2.z * q1.w + q2.x * q1.y - q2.y * q1.x);
}
}  // namespace HAMO

#endif  // HAMO_LINUX_QUATERNION_H
