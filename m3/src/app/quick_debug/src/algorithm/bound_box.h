//
// Created by gaoxiang on 2019/12/12.
//

#ifndef HAMO_LINUX_BOUND_BOX_H
#define HAMO_LINUX_BOUND_BOX_H

#include "algorithm/common.h"
#include "algorithm/constant.h"

namespace HAMO {

template <typename N>
class BoundBox3 {
   public:
    typedef N ValueType;
    typedef Vector3<N> VectorType;

   public:
    BoundBox3();

    BoundBox3(const VectorType &minPt, const VectorType &maxPt);

    BoundBox3(const BoundBox3 &rhs);

   public:
    void Set(const VectorType &minPt, const VectorType &maxPt);

    bool IsValid() const;

    void Reset();

    void Extend(N x);

    void Offset(const VectorType &vec);

    void ExpandBy(N xx, N yy, N zz);

    void ExpandBy(const VectorType &vecPt);

    void ExpandBy(const BoundBox3 &box);

    bool Contains(const VectorType &vecPt) const;

    bool Intersect(const BoundBox3 &rhs) const;

    VectorType GetCenter() const;

   public:
    VectorType v_min_;
    VectorType v_max_;
};

//------------------------------------------------------------------------------
typedef BoundBox3<float> BoundBox3f;
typedef BoundBox3<double> BoundBox3d;

//------------------------------------------------------------------------------

template <typename N>
BoundBox3<N>::BoundBox3()
    : v_min_(Constant<N>::MAX_REAL, Constant<N>::MAX_REAL,
             Constant<N>::MAX_REAL),
      v_max_(-Constant<N>::MAX_REAL, -Constant<N>::MAX_REAL,
             -Constant<N>::MAX_REAL) {}

template <typename N>
BoundBox3<N>::BoundBox3(const VectorType &minpt, const VectorType &maxpt)
    : v_min_(minpt), v_max_(maxpt) {}

template <typename N>
BoundBox3<N>::BoundBox3(const BoundBox3<N> &hrs)
    : v_min_(hrs.v_min_), v_max_(hrs.v_max_) {}

template <typename N>
void BoundBox3<N>::Set(const VectorType &minPt, const VectorType &maxPt) {
    v_min_ = minPt;
    v_max_ = maxPt;
}

template <typename N>
bool BoundBox3<N>::IsValid() const {
    return v_max_.x >= v_min_.x && v_max_.y >= v_min_.y && v_max_.z >= v_min_.z;
}

template <typename N>
void BoundBox3<N>::Reset() {
    v_min_ = VectorType(Constant<N>::MAX_REAL, Constant<N>::MAX_REAL,
                        Constant<N>::MAX_REAL);
    v_max_ = VectorType(-Constant<N>::MAX_REAL, -Constant<N>::MAX_REAL,
                        -Constant<N>::MAX_REAL);
}

template <typename N>
void BoundBox3<N>::Extend(N x) {
    v_min_[0] -= x;
    v_max_[0] += x;
    v_min_[1] -= x;
    v_max_[1] += x;
    v_min_[2] -= x;
    v_max_[2] += x;
}

template <typename N>
void BoundBox3<N>::Offset(const VectorType &vec) {
    v_min_ += vec;
    v_max_ += vec;
}

template <typename N>
void BoundBox3<N>::ExpandBy(N xx, N yy, N zz) {
    if (xx < v_min_[0]) v_min_[0] = xx;
    if (xx > v_max_[0]) v_max_[0] = xx;
    if (yy < v_min_[1]) v_min_[1] = yy;
    if (yy > v_max_[1]) v_max_[1] = yy;
    if (zz < v_min_[2]) v_min_[2] = zz;
    if (zz > v_max_[2]) v_max_[2] = zz;
}

template <typename N>
void BoundBox3<N>::ExpandBy(const VectorType &vec) {
    if (vec[0] < v_min_[0]) v_min_[0] = vec[0];
    if (vec[0] > v_max_[0]) v_max_[0] = vec[0];
    if (vec[1] < v_min_[1]) v_min_[1] = vec[1];
    if (vec[1] > v_max_[1]) v_max_[1] = vec[1];
    if (vec[2] < v_min_[2]) v_min_[2] = vec[2];
    if (vec[2] > v_max_[2]) v_max_[2] = vec[2];
}

template <typename N>
void BoundBox3<N>::ExpandBy(const BoundBox3<N> &box) {
    if (box.v_min_[0] < v_min_[0]) v_min_[0] = box.v_min_[0];
    if (box.v_min_[1] < v_min_[1]) v_min_[1] = box.v_min_[1];
    if (box.v_min_[2] < v_min_[2]) v_min_[2] = box.v_min_[2];
    if (box.v_max_[0] > v_max_[0]) v_max_[0] = box.v_max_[0];
    if (box.v_max_[1] > v_max_[1]) v_max_[1] = box.v_max_[1];
    if (box.v_max_[2] > v_max_[2]) v_max_[2] = box.v_max_[2];
}

template <typename N>
bool BoundBox3<N>::Contains(const VectorType &vecPt) const {
    return (vecPt[0] >= v_min_[0] && vecPt[0] <= v_max_[0] &&
            vecPt[1] >= v_min_[1] && vecPt[1] <= v_max_[1] &&
            vecPt[2] >= v_min_[2] && vecPt[2] <= v_max_[2]);
}

template <typename N>
bool BoundBox3<N>::Intersect(const BoundBox3 &rhs) const {
    if (rhs.v_min_[0] > v_max_[0]) return false;
    if (rhs.v_max_[0] < v_min_[0]) return false;
    if (rhs.v_min_[1] > v_max_[1]) return false;
    if (rhs.v_max_[1] < v_min_[1]) return false;
    if (rhs.v_min_[2] > v_max_[2]) return false;
    if (rhs.v_max_[2] < v_min_[2]) return false;

    return true;
};

template <typename N>
Vector3<N> BoundBox3<N>::GetCenter() const {
    ValueType half = (ValueType)0.5;
    VectorType vec = v_min_ + v_max_;
    vec[0] *= half;
    vec[1] *= half;
    vec[2] *= half;

    return vec;
}

}  // namespace HAMO
#endif  // HAMO_LINUX_BOUND_BOX_H
