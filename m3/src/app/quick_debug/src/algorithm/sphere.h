//
// Created by gaoxiang on 2019/12/13.
//

#ifndef HAMO_LINUX_SPHERE_H
#define HAMO_LINUX_SPHERE_H

#include "algorithm/common.h"

namespace HAMO {

template <typename N>
class Sphere {
   public:
    typedef N ValueType;
    typedef Eigen::Matrix<N, 3, 1> VectorType;

   public:
    Sphere();

    Sphere(const VectorType &center, N radius);

    Sphere(const Sphere &rhs);

   public:
    void SetConstant(N val);

    bool IsValid() const;

    bool ContainPoint(const VectorType &point) const;

    bool Intersect(const Sphere<N> &sphere) const;

    void ExpandBy(const VectorType &point);

    void ExpandBy(const Sphere<N> &sphere);

   public:
    VectorType center_;
    N radius_;
};

//------------------------------------------------------------------------------
typedef Sphere<float> Spheref;
typedef Sphere<double> Sphered;

//------------------------------------------------------------------------------

template <typename N>
Sphere<N>::Sphere() : radius_(-1.0) {}

template <typename N>
Sphere<N>::Sphere(const VectorType &center, N radius)
    : center_(center), radius_(radius) {}

template <typename N>
Sphere<N>::Sphere(const Sphere<N> &rhs)
    : center_(rhs.center_), radius_(rhs.radius_) {}

template <typename N>
bool Sphere<N>::ContainPoint(const VectorType &point) const {
    N dist = center_.LengthSquare(point);
    return (dist < radius_ * radius_);
}

template <typename N>
bool Sphere<N>::IsValid() const {
    return (radius_ >= ValueType(0));
}

template <typename N>
bool Sphere<N>::Intersect(const Sphere<N> &bs) const {
    if (IsValid() && bs.IsValid()) {
        VectorType vec = center_ - bs._center;
        N len = radius_ + bs.radius_;
        return (vec.LengthSquare() < len * len);
    }
    return false;
}

template <typename N>
void Sphere<N>::ExpandBy(const VectorType &point) {
    if (IsValid()) {
        VectorType dv = point - center_;

        N r = dv.Length();
        if (r > radius_) {
            N dr = (r - radius_) * ValueType(0.5);
            center_ += dv * (dr / r);
            radius_ += dr;
        }
    } else {
        center_ = point;
        radius_ = ValueType(0);
    }
}

template <typename N>
void Sphere<N>::ExpandBy(const Sphere<N> &sh) {
    if (!sh.IsValid()) return;

    if (!IsValid()) {
        center_ = sh.center_;
        radius_ = sh.radius_;

        return;
    }

    N d = (center_ - sh.center_).Length();

    if (d + sh.radius_ <= radius_) {
        return;
    }

    if (d + radius_ <= sh.radius_) {
        center_ = sh.center_;
        radius_ = sh.radius_;
        return;
    }

    N new_radius = (radius_ + d + sh.radius_) * ValueType(0.5);
    N ratio = (new_radius - radius_) / d;

    center_ += (sh.center_ - center_) * ratio;
    radius_ = new_radius;
}

}  // namespace HAMO

#endif  // HAMO_LINUX_SPHERE_H
