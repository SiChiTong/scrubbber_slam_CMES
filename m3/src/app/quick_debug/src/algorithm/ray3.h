//
// Created by gaoxiang on 2019/12/13.
//

#ifndef HAMO_LINUX_RAY3_H
#define HAMO_LINUX_RAY3_H

#include "algorithm/common.h"
#include "algorithm/matrix44.h"
#include "algorithm/mc_math.h"
#include "algorithm/vector3.h"

namespace HAMO {

template <typename N>
class Ray3 {
   public:
    typedef N ValueType;
    typedef Vector3<N> VectorType;
    typedef Matrix4x4<N> M44Type;

   public:
    Ray3();

    Ray3(const VectorType &org, const VectorType &dir);

    Ray3(const Ray3<N> &rhs);

   public:
    void Normalize();

    void Transform(const M44Type &matrix);

    VectorType GetPoint(N t) const;

   public:
    VectorType origin_;
    VectorType direction_;
};

//------------------------------------------------------------------------------
typedef Ray3<float> Ray3f;
typedef Ray3<double> Ray3d;

//------------------------------------------------------------------------------

template <typename N>
Ray3<N>::Ray3() {}

template <typename N>
Ray3<N>::Ray3(const VectorType &org, const VectorType &dir) {
    origin_ = org;
    direction_ = dir;
}

template <typename N>
Ray3<N>::Ray3(const Ray3<N> &rhs) {
    origin_ = rhs.origin_;
    direction_ = rhs.direction_;
}

template <typename N>
void Ray3<N>::Normalize() {
    N xx = direction_.x * direction_.x;
    N yy = direction_.y * direction_.y;
    N zz = direction_.z * direction_.z;

    N fLength = Math<N>::Sqrt(xx + yy + zz);
    if (fLength > ValueType(0)) {
        N fInvLength = ValueType(1) / fLength;
        direction_.x *= fInvLength;
        direction_.y *= fInvLength;
        direction_.z *= fInvLength;
    }
}

template <typename N>
Vector3<N> Ray3<N>::GetPoint(N t) const {
    return direction_ * t + origin_;
}

template <typename N>
void Ray3<N>::Transform(const M44Type &matrix) {
    origin_ = matrix.TransformCoord(origin_);
    direction_ = matrix.TransformNormal(direction_);
}

}  // namespace HAMO

#endif  // HAMO_LINUX_RAY3_H
